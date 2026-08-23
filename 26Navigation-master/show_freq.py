#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from rclpy.time import Time
import sys
import os
import threading
import signal
from collections import defaultdict
import time

# 导入常用消息类型
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from nav_msgs.msg import Odometry, Path, OccupancyGrid

LOG_FILE = "topic_frequencies.log"

# 话题和类型映射，可根据你的需求修改
TOPICS_TO_MONITOR = {
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/odom': 'nav_msgs/msg/Odometry',
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/imu/data': 'sensor_msgs/msg/Imu',
    '/map': 'nav_msgs/msg/OccupancyGrid',
    '/global_plan': 'nav_msgs/msg/Path',
    '/local_plan': 'nav_msgs/msg/Path',
    '/joint_states': 'sensor_msgs/msg/JointState',
    '/livox/lidar/pointcloud': 'sensor_msgs/msg/PointCloud2'
}

# ROS2 消息类型映射（补充常用类型）
from geometry_msgs.msg import Twist, Pose, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState  # 加入导入
MSG_TYPE_MAP = {
    'std_msgs/msg/String': String,
    'std_msgs/msg/Float32': Float32,
    'sensor_msgs/msg/Imu': Imu,
    'sensor_msgs/msg/LaserScan': LaserScan,
    'sensor_msgs/msg/PointCloud2': PointCloud2,
    'sensor_msgs/msg/JointState': JointState,  # 新加
    'nav_msgs/msg/Odometry': Odometry,
    'nav_msgs/msg/Path': Path,
    'nav_msgs/msg/OccupancyGrid': OccupancyGrid,
    'geometry_msgs/msg/Twist': Twist,
    'geometry_msgs/msg/Pose': Pose,
    'geometry_msgs/msg/PoseStamped': PoseStamped,
    'geometry_msgs/msg/TransformStamped': TransformStamped
}
class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor')
        self.topic_counts = defaultdict(int)
        self.topic_last_time = defaultdict(lambda: self.get_clock().now())
        self.topic_freq = defaultdict(float)
        self.lock = threading.Lock()

        # 日志文件
        self.log_file = open(LOG_FILE, 'w')

        # 订阅话题
        for topic, msg_type_str in TOPICS_TO_MONITOR.items():
            if msg_type_str not in MSG_TYPE_MAP:
                raise TypeError(f"Unknown message type for topic {topic}: {msg_type_str}")
            msg_type = MSG_TYPE_MAP[msg_type_str]
            try:
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, t=topic: self.callback(msg, t),
                    QoSProfile(depth=10)
                )
            except Exception as e:
                raise RuntimeError(f"Failed to subscribe to topic {topic}: {e}")

        # 清屏刷新频率
        self.timer = self.create_timer(1.0, self.print_frequencies)

    def callback(self, msg, topic_name):
        now = self.get_clock().now()
        with self.lock:
            dt = (now - self.topic_last_time[topic_name]).nanoseconds * 1e-9
            if dt > 0:
                self.topic_freq[topic_name] = 1.0 / dt
            self.topic_last_time[topic_name] = now
            self.topic_counts[topic_name] += 1

    def print_frequencies(self):
        os.system('clear')  # Linux/macOS清屏
        lines = ["Topic frequencies (Hz):"]
        with self.lock:
            for topic, freq in self.topic_freq.items():
                lines.append(f"{topic:30s}: {freq:.2f} Hz")

        output = "\n".join(lines)
        print(output)
        self.log_file.write(output + "\n")
        self.log_file.flush()

    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitor()

    def signal_handler(sig, frame):
        print("Shutting down...")
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()