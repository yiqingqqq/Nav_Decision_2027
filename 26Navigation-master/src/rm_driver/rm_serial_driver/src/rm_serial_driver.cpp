#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "uart.hpp"
#include <memory>
#include <limits>
#include <mutex>
#include <thread>
#include <atomic>

class RMSerialDriver : public rclcpp::Node
{
public:
    RMSerialDriver()
        : Node("rm_serial_driver"),
          cmd_vx(0.0f), cmd_vy(0.0f), cmd_wz(0.0f),
          cmd_yaw(0.0f), cmd_pitch(0.0f),
          cmd_decision_wz(0.0f)
    {
        vx_limit_        = this->declare_parameter("vx_limit", 1.0f);
        vy_limit_        = this->declare_parameter("vy_limit", 1.0f);
        wz_limit_        = this->declare_parameter("wz_limit", 1.0f);
        cmd_vel_timeout_ = this->declare_parameter("cmd_vel_timeout", 0.5);

        std::string port = this->declare_parameter<std::string>("port", "/dev/ttyACM0");

        last_cmd_vel_time_ = this->now();
        last_wz_time_      = this->now();

        driver_ = std::make_shared<Driver>(port, BaudRate::BAUD_115200);
        if (driver_->open() != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
        }

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&RMSerialDriver::cmdVelCallback, this, std::placeholders::_1));

        // /cmd_gimbal: data=[yaw, pitch]
        gimbal_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/cmd_gimbal", 10,
            std::bind(&RMSerialDriver::gimbalCmdCallback, this, std::placeholders::_1));

        // /cmd_control_flags: UInt8  0xFF=开火  0x00=不开火
        flags_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/cmd_control_flags", 10,
            std::bind(&RMSerialDriver::flagsCallback, this, std::placeholders::_1));

        wz_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/cmd_vel_wz", 10,
            std::bind(&RMSerialDriver::wz_CmdCallback, this, std::placeholders::_1));

        pub_gimbal_state_        = this->create_publisher<std_msgs::msg::Float32MultiArray>("feedback_gimbal_state", 10);
        pub_robot_hp_            = this->create_publisher<std_msgs::msg::UInt16>("feedback_robot_hp", 10);
        pub_robot_id_            = this->create_publisher<std_msgs::msg::UInt8>("feedback_robot_id", 10);
        pub_game_progress_       = this->create_publisher<std_msgs::msg::UInt8>("feedback_game_progress", 10);
        pub_hp_deduction_reason_ = this->create_publisher<std_msgs::msg::UInt8>("feedback_hp_deduction_reason", 10);

        recv_running_ = true;
        recv_thread_ = std::thread(&RMSerialDriver::recvLoop, this);

        tx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RMSerialDriver::sendSerialData, this));
    }

    ~RMSerialDriver()
    {
        if (driver_) {
            Control_Flag_t zero_flag{};
            zero_flag.fire = 0x00;
            RCLCPP_INFO(this->get_logger(), "[Shutdown] Sending zero velocity before closing.");
            driver_->nav_data(0.f, 0.f, 0.f, cmd_yaw, cmd_pitch, zero_flag);
        }
        recv_running_ = false;
        if (recv_thread_.joinable()) recv_thread_.join();
        if (driver_) driver_->close();
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        cmd_vx = std::clamp(static_cast<float>(msg->linear.x),  -vx_limit_, vx_limit_);
        cmd_vy = std::clamp(static_cast<float>(msg->linear.y),  -vy_limit_, vy_limit_);
        cmd_wz = std::clamp(static_cast<float>(msg->angular.z), -wz_limit_, wz_limit_);
        last_cmd_vel_time_ = this->now();
        RCLCPP_DEBUG(this->get_logger(),
            "[cmd_vel] vx=%.3f  vy=%.3f  wz=%.3f", cmd_vx, cmd_vy, cmd_wz);
    }

    void gimbalCmdCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(),
                "[cmd_gimbal] 需要2个元素 [yaw, pitch]，实际收到 %zu 个", msg->data.size());
            return;
        }
        std::lock_guard<std::mutex> lk(mutex_);
        cmd_yaw   = msg->data[0];
        cmd_pitch = msg->data[1];
        RCLCPP_DEBUG(this->get_logger(),
            "[cmd_gimbal] yaw=%.3f  pitch=%.3f", cmd_yaw, cmd_pitch);
    }

    void flagsCallback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        control_flag_.fire = msg->data; // 0xFF=开火, 0x00=不开火
        RCLCPP_DEBUG(this->get_logger(), "[flags] fire=0x%02X", control_flag_.fire);
    }

    void wz_CmdCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        cmd_decision_wz = std::clamp(msg->data, -wz_limit_, wz_limit_);
        last_wz_time_ = this->now();
        RCLCPP_DEBUG(this->get_logger(), "[cmd_decision_wz] wz=%.3f (override)", cmd_decision_wz);
    }

    void sendSerialData()
    {
        if (!driver_) return;
        std::lock_guard<std::mutex> data_lk(mutex_);

        if (cmd_vel_timeout_ > 0.0) {
            double dt_vel = (this->now() - last_cmd_vel_time_).seconds();
            if (dt_vel > cmd_vel_timeout_) {
                if (cmd_vx != 0.f || cmd_vy != 0.f)
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[Timeout] No /cmd_vel for %.2f s, zeroing vx/vy/wz.", dt_vel);
                cmd_vx = 0.f;
                cmd_vy = 0.f;
                cmd_wz = 0.f;
            }

            double dt_wz = (this->now() - last_wz_time_).seconds();
            if (dt_wz > cmd_vel_timeout_) {
                if (cmd_decision_wz != 0.f)
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[Timeout] No /cmd_vel_wz for %.2f s, zeroing wz.", dt_wz);
                cmd_decision_wz = 0.f;
            }
        }

        driver_->nav_data(cmd_vx, cmd_vy, cmd_decision_wz,
                          cmd_yaw, cmd_pitch, control_flag_);
        std::cout << "\033[32m[TX] yaw=" 
                << cmd_yaw * 180.0f / M_PI
                << " deg, pitch=" 
                << cmd_pitch * 180.0f / M_PI
                << " deg\033[0m" 
                << std::endl;
    }

    void recvLoop()
    {
        while (recv_running_) {
            float    yaw = 0.f, pitch = 0.f;
            uint16_t robot_hp             = 0;
            uint8_t  robot_id             = 0;
            uint8_t  game_progress        = 0;
            uint8_t  hp_deduction_reason  = 0;

            int ret = driver_->recv_nav_data(
                yaw, pitch,
                robot_hp, robot_id,
                game_progress, hp_deduction_reason,
                50 /* ms timeout */);

            if (ret == 0) {
                RCLCPP_INFO(this->get_logger(),
                    "\033[35m[RX] yaw=%.3f  pitch=%.3f  |  "
                    "robot_hp=%u  robot_id=%u  "
                    "game_progress=%u  hp_deduction_reason=%u\033[0m",
                    yaw, pitch,
                    robot_hp, (uint16_t)robot_id,
                    game_progress, hp_deduction_reason);

                // 云台反馈 [yaw, pitch]
                std_msgs::msg::Float32MultiArray gimbal_msg;
                gimbal_msg.data = {yaw, pitch};
                pub_gimbal_state_->publish(gimbal_msg);

                std_msgs::msg::UInt16 hp_msg;
                hp_msg.data = robot_hp;
                pub_robot_hp_->publish(hp_msg);

                std_msgs::msg::UInt8 id_msg;
                id_msg.data = robot_id;
                pub_robot_id_->publish(id_msg);

                std_msgs::msg::UInt8 gp_msg;
                gp_msg.data = game_progress;
                pub_game_progress_->publish(gp_msg);

                std_msgs::msg::UInt8 hdr_msg;
                hdr_msg.data = hp_deduction_reason;
                pub_hp_deduction_reason_->publish(hdr_msg);

            } else if (ret == -2) {
                // 超时，继续循环
            } else if (ret == -3) {
                static int crc_warn_cnt = 0;
                if (++crc_warn_cnt % 20 == 1)
                    RCLCPP_WARN(this->get_logger(), "[RX] CRC 校验失败，丢弃本帧");
            } else {
                static int err_cnt = 0;
                if (++err_cnt % 20 == 1)
                    RCLCPP_ERROR(this->get_logger(), "[RX] 接收错误，code=%d", ret);
            }
        }
    }

    std::shared_ptr<Driver> driver_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr         cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr  gimbal_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr              flags_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            wz_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_gimbal_state_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr            pub_robot_hp_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             pub_robot_id_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             pub_game_progress_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             pub_hp_deduction_reason_;

    std::thread       recv_thread_;
    std::atomic<bool> recv_running_{false};
    rclcpp::TimerBase::SharedPtr tx_timer_;

    std::mutex     mutex_;
    float          cmd_vx, cmd_vy, cmd_wz, cmd_decision_wz;
    float          cmd_yaw, cmd_pitch;
    Control_Flag_t control_flag_{};

    float  vx_limit_, vy_limit_, wz_limit_;
    double cmd_vel_timeout_;
    rclcpp::Time last_cmd_vel_time_;
    rclcpp::Time last_wz_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RMSerialDriver>());
    rclcpp::shutdown();
    return 0;
}