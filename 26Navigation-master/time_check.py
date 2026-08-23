#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

class TimeSyncChecker(Node):
    def __init__(self):
        super().__init__('time_sync_checker')

        # QoS 设置
        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        qos_default = QoSProfile(depth=10)  # 默认 RELIABLE

        # 订阅话题
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_sensor)
        self.sub_livox = self.create_subscription(PointCloud2, '/livox/lidar/pointcloud', self.cb_livox, qos_sensor)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, qos_default)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.cb_imu, qos_default)

        # 保存最近时间戳
        self.ts_scan = None
        self.ts_livox = None
        self.ts_odom = None
        self.ts_imu = None

    def cb_scan(self, msg):
        self.ts_scan = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.print_diff()

    def cb_livox(self, msg):
        self.ts_livox = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.print_diff()

    def cb_odom(self, msg):
        self.ts_odom = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.print_diff()

    def cb_imu(self, msg):
        self.ts_imu = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.print_diff()

    def print_diff(self):
        # 确保四个时间戳都收到过
        if None in [self.ts_scan, self.ts_livox, self.ts_odom, self.ts_imu]:
            return

        scan_odom = self.ts_scan - self.ts_odom
        scan_imu = self.ts_scan - self.ts_imu
        scan_livox = self.ts_scan - self.ts_livox

        self.get_logger().info(
            f"scan-odom: {scan_odom*1000:.2f} ms | "
            f"scan-imu: {scan_imu*1000:.2f} ms | "
            f"scan-livox: {scan_livox*1000:.2f} ms"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TimeSyncChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()