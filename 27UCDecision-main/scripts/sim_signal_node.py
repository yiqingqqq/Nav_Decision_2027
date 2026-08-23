#!/usr/bin/env python3
"""RMUC 2026 哨兵仿真信号节点

模拟 ROS2 环境供 BT 决策节点运行:
  - mock Nav2 NavigateToPose action server
  - TF publisher (base_link → map)
  - 游戏信号 / feedback_game_progress, / feedback_robot_hp
  - 响应 /tmp/sim_* 触发文件实现阶段转换

触发文件:
  touch /tmp/sim_game_run     → GameProgress=4 (比赛开始)
  touch /tmp/sim_game_stop    → GameProgress=0 (比赛暂停)
  touch /tmp/sim_hp_low       → HP=70    (触发补给撤退)
  touch /tmp/sim_hp_mid       → HP=200   (部分恢复，滞回中)
  touch /tmp/sim_hp_high      → HP=600   (完全恢复)
  touch /tmp/sim_nav_fail     → 下次导航 ABORT (模拟导航失败)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_msgs.action import NavigateToPose
from std_msgs.msg import UInt8, UInt16, Float32
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

import os
import time
import threading

C = '\033[96m'
G = '\033[92m'
Y = '\033[93m'
R = '\033[91m'
BOLD = '\033[1m'
NC = '\033[0m'


class MockNav2ActionServer(Node):
    """Simulates Nav2's NavigateToPose action server."""

    def __init__(self):
        super().__init__('mock_nav2')
        self._action_server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose',
            execute_callback=self._execute_cb,
            callback_group=ReentrantCallbackGroup(),
        )
        self._fail_countdown = 0
        # Check for nav_fail trigger file periodically
        self._trig_timer = self.create_timer(0.5, self._check_triggers)
        print(f'{C}[MockNav2] Ready on /navigate_to_pose{NC}')

    def _check_triggers(self):
        nf = '/tmp/sim_nav_fail'
        if os.path.exists(nf):
            os.unlink(nf)
            self._fail_countdown = 4  # fail next 4 goals to exhaust 2 retries + fallback
            print(f'{R}[MockNav2] ← triggered: will fail next {self._fail_countdown} goals{NC}', flush=True)

    def _execute_cb(self, goal_handle):
        req = goal_handle.request
        px = req.pose.pose.position.x
        py = req.pose.pose.position.y
        print(f'{C}[MockNav2] Goal ({px:.1f}, {py:.1f}){NC}', flush=True)

        if self._fail_countdown > 0:
            self._fail_countdown -= 1
            print(f'{R}[MockNav2] → ABORT (countdown={self._fail_countdown}){NC}', flush=True)
            goal_handle.abort()
            return NavigateToPose.Result()

        # Simulate travel
        dist = ((px - 5.0) ** 2 + (py - 5.0) ** 2) ** 0.5
        travel = max(0.5, min(dist * 0.15, 3.0))
        fb = NavigateToPose.Feedback()
        for frac, rem in [(0.33, dist * 0.6), (0.66, dist * 0.2), (1.0, 0.0)]:
            fb.distance_remaining = rem
            goal_handle.publish_feedback(fb)
            time.sleep(travel / 3.0)

        print(f'{G}[MockNav2] → SUCCESS{NC}', flush=True)
        goal_handle.succeed()
        return NavigateToPose.Result()


class TFPublisher(Node):
    """Publish base_link → map at 10 Hz."""

    def __init__(self, x=9.0, y=6.0):
        super().__init__('mock_tf')
        self._pub = self.create_publisher(TFMessage, '/tf', 10)
        self._x, self._y = x, y
        self._timer = self.create_timer(0.1, self._publish)

    def _publish(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        msg = TFMessage()
        msg.transforms.append(t)
        self._pub.publish(msg)


class SignalPublisher(Node):
    """Publish game progress and HP, watch trigger files."""

    def __init__(self):
        super().__init__('mock_signals')
        self._game_pub = self.create_publisher(UInt8, '/feedback_game_progress', 10)
        self._hp_pub = self.create_publisher(UInt16, '/feedback_robot_hp', 10)
        self._spin_sub = self.create_subscription(
            Float32, '/cmd_vel_wz', self._spin_cb, 10)

        self._game_progress = 0
        self._hp = 500
        self._timer = self.create_timer(0.1, self._tick)
        print(f'{C}[SignalPub] Publishing /feedback_game_progress + /feedback_robot_hp{NC}')

    def _spin_cb(self, msg):
        pass  # consume spin commands

    def _tick(self):
        self._check_triggers()
        gp = UInt8(); gp.data = self._game_progress
        hp = UInt16(); hp.data = self._hp
        self._game_pub.publish(gp)
        self._hp_pub.publish(hp)

    def _check_triggers(self):
        checks = {
            'sim_game_run':  lambda: self._set_game(4),
            'sim_game_stop': lambda: self._set_game(0),
            'sim_hp_low':    lambda: self._set_hp(70,  'HP=70  (低于 retreat=110, 触发补给)'),
            'sim_hp_mid':    lambda: self._set_hp(200, 'HP=200 (低于 recover=400, 滞回中)'),
            'sim_hp_high':   lambda: self._set_hp(600, 'HP=600 (超过 recover=400, 恢复巡航)'),
        }
        for fname, action in checks.items():
            path = f'/tmp/{fname}'
            if os.path.exists(path):
                os.unlink(path)
                action()

    def _set_game(self, v: int):
        self._game_progress = v
        label = '比赛开始' if v == 4 else '比赛暂停/未开始'
        print(f'{G}[Signal] GameProgress → {v} ({label}){NC}', flush=True)

    def _set_hp(self, v: int, label: str):
        self._hp = v
        print(f'{Y}[Signal] {label}{NC}', flush=True)


def main():
    rclpy.init()
    ex = MultiThreadedExecutor()
    ex.add_node(MockNav2ActionServer())
    ex.add_node(TFPublisher())
    ex.add_node(SignalPublisher())

    print(f'{BOLD}══════════════════════════════════════════════════{NC}')
    print(f'{BOLD}  仿真信号节点已启动{NC}')
    print(f'{BOLD}  触发文件: touch /tmp/sim_*{NC}')
    print(f'{BOLD}══════════════════════════════════════════════════{NC}')
    print(flush=True)

    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
