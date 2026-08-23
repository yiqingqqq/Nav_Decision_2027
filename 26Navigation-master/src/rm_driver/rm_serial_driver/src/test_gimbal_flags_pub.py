# 用于测试 RMSerialDriver，发布云台位姿到 /gimbal_cmd 和控制标志位到 /control_flags
# 可直接运行：python3 test_gimbal_flags_pub.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt8


class GimbalFlagsTestPublisher(Node):
    def __init__(self):
        super().__init__('gimbal_flags_test_pub')

        self.gimbal_pub = self.create_publisher(Float32MultiArray, '/cmd_gimbal', 10)
        self.flags_pub  = self.create_publisher(UInt8, '/cmd_control_flags', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)  # 每秒发布一次

    def timer_callback(self):
        # ---- 云台位姿 [yaw, pitch, depth] ----
        gimbal_msg = Float32MultiArray()
        gimbal_msg.data = [-2.19, -0.38, 0.0]  
        self.gimbal_pub.publish(gimbal_msg)
        self.get_logger().info(
            f'[gimbal_cmd] yaw={gimbal_msg.data[0]}, '
            f'pitch={gimbal_msg.data[1]}, depth={gimbal_msg.data[2]}'
        )

        # ---- 控制标志位 ----
        # bit0=auto_aim, bit1=auto_shoot, bit2=exit_auto_aim, bit3=exit_auto_shoot
        # 示例：auto_aim=1, auto_shoot=1 → 0b00000011 = 3
        flags_msg = UInt8()
        flags_msg.data = 0x03
        self.flags_pub.publish(flags_msg)
        self.get_logger().info(
            f'[control_flags] 0x{flags_msg.data:02X}  '
            f'auto_aim={flags_msg.data & 0x01}, '
            f'auto_shoot={(flags_msg.data >> 1) & 0x01}, '
            f'exit_auto_aim={(flags_msg.data >> 2) & 0x01}, '
            f'exit_auto_shoot={(flags_msg.data >> 3) & 0x01}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GimbalFlagsTestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

## 运行
# python3 test_gimbal_flags_pub.py

## 标志位说明
# 0x01 = auto_aim only
# 0x02 = auto_shoot only
# 0x03 = auto_aim + auto_shoot
# 0x04 = exit_auto_aim
# 0x08 = exit_auto_shoot