# 用于测试 RMSerialDriver，发布 wz 到 /cmd_vel_wz
# 可直接运行：python3 test_cmd_vel_wz_pub.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class CmdVelWzPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_wz_pub')
        self.publisher_ = self.create_publisher(Float32, '/cmd_vel_wz', 10)
        self.publish_period = 0.3
        self.target_wz = 5.0
        self.ramp_rate = 1.0  # rad/s^2，每秒变化量
        self.current_wz = 0.0
        self.timer = self.create_timer(self.publish_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        step = self.ramp_rate * self.publish_period
        if self.current_wz < self.target_wz:
            self.current_wz = min(self.current_wz + step, self.target_wz)

        msg.data = self.current_wz  # wz (rad/s)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: wz={msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWzPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

## 运行
# python3 test_cmd_vel_wz_pub.py