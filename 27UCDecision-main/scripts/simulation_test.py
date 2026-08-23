#!/usr/bin/env python3
"""
RMUC 2026 哨兵决策系统 — 仿真测试脚本

模仿仿真系统，通过对 BT 节点发送模拟信号，测试：
1. 条件转换（低血量→补给、受击→撤退、导航失败→fallback）
2. 三套巡航策略（defense / balanced / aggressive）的行为差异

运行方式：
  ros2 run rm_behavior_tree simulation_test
  或直接 source install/setup.bash 后 python3 scripts/simulation_test.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from nav2_msgs.action import NavigateToPose
from std_msgs.msg import UInt8, UInt16, Float32
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_msgs.msg import TFMessage

import subprocess
import signal
import time
import sys
import os
import threading


# ── Colors for terminal output ──
class Color:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    END = '\033[0m'


def banner(text, ch='='):
    """Print a visible section banner."""
    width = 72
    print(f'\n{Color.BOLD}{Color.CYAN}{ch * width}{Color.END}')
    print(f'{Color.BOLD}{Color.CYAN}{ch * 3}  {text}{Color.END}')
    print(f'{Color.BOLD}{Color.CYAN}{ch * width}{Color.END}\n')


def phase(text):
    """Print a phase indicator."""
    print(f'\n{Color.BOLD}{Color.GREEN}>>> {text}{Color.END}')


def signal_msg(label, value):
    """Print a signal being sent."""
    print(f'{Color.BOLD}{Color.YELLOW}[信号] {label} → {value}{Color.END}')


def wait(s):
    time.sleep(s)


# ═══════════════════════════════════════════════════════════════
#  Mock Nav2 Action Server
# ═══════════════════════════════════════════════════════════════

class MockNav2ActionServer(Node):
    """Simulates Nav2's NavigateToPose action server."""

    def __init__(self):
        super().__init__('mock_nav2')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self._fail_next = False
        self._force_abort = False
        self.get_logger().info('[Nav2Mock] Action server ready on /navigate_to_pose')

    def set_fail_next(self, fail: bool = True):
        self._fail_next = fail

    def execute_callback(self, goal_handle):
        """Simulate navigation: wait, then succeed or fail as directed."""
        req = goal_handle.request
        px = req.pose.pose.position.x
        py = req.pose.pose.position.y
        self.get_logger().info(
            f'[Nav2Mock] Received goal -> ({px:.2f}, {py:.2f})')

        if self._fail_next:
            self._fail_next = False
            self.get_logger().warn('[Nav2Mock] Force ABORT (simulated failure)')
            goal_handle.abort()
            result = NavigateToPose.Result()
            return result

        # Simulate navigation time proportional to distance
        dist = ((px - 5.0)**2 + (py - 5.0)**2)**0.5
        travel_time = max(0.5, dist * 0.15)

        # Publish feedback
        feedback_msg = NavigateToPose.Feedback()
        feedback_msg.distance_remaining = dist
        for remaining in [dist * 0.7, dist * 0.3, 0.0]:
            feedback_msg.distance_remaining = remaining
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(travel_time / 3.0)

        self.get_logger().info(f'[Nav2Mock] Goal SUCCEEDED at ({px:.2f}, {py:.2f})')
        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result


# ═══════════════════════════════════════════════════════════════
#  TF Publisher
# ═══════════════════════════════════════════════════════════════

class TFPublisher(Node):
    """Publishes a static base_link → map transform so the BT can locate itself."""

    def __init__(self, x=5.0, y=5.0, z=0.0):
        super().__init__('mock_tf_publisher')
        self._pub = self.create_publisher(TFMessage, '/tf', 10)
        self._x = x
        self._y = y
        self._z = z
        self._timer = self.create_timer(0.1, self.publish_tf)

    def update_pose(self, x: float, y: float):
        self._x = x
        self._y = y

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = self._z
        t.transform.rotation.w = 1.0

        msg = TFMessage()
        msg.transforms.append(t)
        self._pub.publish(msg)


# ═══════════════════════════════════════════════════════════════
#  Signal Publisher (Game Status + HP)
# ═══════════════════════════════════════════════════════════════

class SignalPublisher(Node):
    """Publishes game progress (UInt8) and robot HP (UInt16)."""

    def __init__(self):
        super().__init__('mock_signal_publisher')
        self._game_pub = self.create_publisher(UInt8, '/feedback_game_progress', 10)
        self._hp_pub = self.create_publisher(UInt16, '/feedback_robot_hp', 10)
        self._spin_sub = self.create_subscription(
            Float32, '/cmd_vel_wz', self.spin_callback, 10)

        self._game_progress = 0
        self._hp = 400
        self._last_spin_cmd = 0.0

        # Publish at 10 Hz
        self._timer = self.create_timer(0.1, self.publish_signals)

    def spin_callback(self, msg):
        self._last_spin_cmd = msg.data

    def set_game_progress(self, value: int):
        self._game_progress = value
        signal_msg(f'GameProgress → {value}', f'{"RUNNING" if value == 4 else "NOT_RUNNING"}')

    def set_hp(self, value: int):
        self._hp = value
        status = 'normal' if value > 110 else 'LOW_HP_RETREAT' if value <= 110 else 'unknown'
        signal_msg(f'HP → {value}', status)

    def publish_signals(self):
        gp = UInt8()
        gp.data = self._game_progress
        self._game_pub.publish(gp)

        hp = UInt16()
        hp.data = self._hp
        self._hp_pub.publish(hp)

    def get_spin_state(self) -> str:
        return 'SPINNING' if self._last_spin_cmd > 0.5 else 'stopped'


# ═══════════════════════════════════════════════════════════════
#  BT Process Manager
# ═══════════════════════════════════════════════════════════════

class BTProcess:
    """Manages the behavior tree C++ node as a subprocess."""

    def __init__(self, workspace_dir: str, preset: str, alliance: str = 'red'):
        self._workspace = workspace_dir
        self._preset = preset
        self._alliance = alliance
        self._proc: subprocess.Popen | None = None

    def start(self):
        banner(f'启动 BT — preset={self._preset}', '-')
        env = os.environ.copy()
        env['RMUC_PRESET'] = self._preset
        env['RMUC_ALLIANCE'] = self._alliance

        # Source ROS2 + workspace
        setup_cmd = (
            f'cd {self._workspace} && '
            f'source /opt/ros/humble/setup.bash && '
            f'source install/setup.bash && '
            f'ros2 launch rm_behavior_tree rm_behavior_tree.launch.py '
            f'style:=rmuc_2026.xml '
            f'preset:={self._preset} '
            f'alliance:={self._alliance} '
            f'use_sim_time:=True'
        )

        self._proc = subprocess.Popen(
            ['bash', '-c', setup_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,
        )

        # Wait a moment for initialization
        time.sleep(1.5)
        self._check_alive()
        print(f'{Color.GREEN}[BT] Started (pid={self._proc.pid}){Color.END}')

    def stop(self):
        if self._proc and self._proc.poll() is None:
            banner('停止 BT', '-')
            os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                self._proc.wait()
            print(f'{Color.GREEN}[BT] Stopped{Color.END}')
        # Extra cleanup for stray processes
        subprocess.run(
            'pkill -f "rm_behavior_tree" 2>/dev/null; '
            'pkill -f "btcpp_ros" 2>/dev/null',
            shell=True)
        time.sleep(0.5)

    def _check_alive(self):
        if self._proc and self._proc.poll() is not None:
            stdout, _ = self._proc.communicate()
            print(f'{Color.RED}[BT] Process died! stdout:{Color.END}\n{stdout}')
            raise RuntimeError('BT process exited prematurely')

    def read_log(self, lines: int = 3):
        """Try to read recent BT log output."""
        try:
            import select
            if self._proc and self._proc.stdout:
                ready, _, _ = select.select([self._proc.stdout], [], [], 0.1)
                if ready:
                    output_lines = []
                    for _ in range(lines * 3):
                        line = self._proc.stdout.readline()
                        if line:
                            output_lines.append(line.strip())
                        else:
                            break
                    return output_lines
        except Exception:
            pass
        return []


# ═══════════════════════════════════════════════════════════════
#  Test Scenarios
# ═══════════════════════════════════════════════════════════════

def run_preset_test(
    bt: BTProcess,
    mock_nav: MockNav2ActionServer,
    signal_pub: SignalPublisher,
    tf_pub: TFPublisher,
    preset_name: str,
    desc: str,
):
    """Run a full test cycle for one preset."""
    banner(f'策略: {preset_name} — {desc}', '=')
    phase('初始化: 比赛未开始')
    signal_pub.set_game_progress(0)  # Not running
    signal_pub.set_hp(400)
    wait(2.0)

    phase('比赛开始 (GameProgress=4)')
    signal_pub.set_game_progress(4)
    wait(5.0)  # Let it settle into first patrol waypoint

    phase('切换到防守预设...')
    # The BT needs to be restarted with the new preset
    # Already handled by the test runner

    phase('验证巡航路线：观察 SendGoal 输出的坐标')

    # Let it cycle through patrol route a few times
    wait(10.0)

    phase('低血量测试: HP → 100 (< 110 补给阈值)')
    signal_pub.set_hp(100)
    wait(6.0)

    phase('血量恢复: HP → 300 (仍在恢复阈值 400 以下，应继续补给)')
    signal_pub.set_hp(300)
    wait(4.0)

    phase('血量恢复: HP → 500 (> 400 恢复阈值，返回巡航)')
    signal_pub.set_hp(500)
    wait(6.0)

    phase('导航失败测试')
    mock_nav.set_fail_next(True)
    wait(8.0)

    phase('恢复正常')
    wait(4.0)

    phase(f'完成 {preset_name} 测试')
    bt.stop()


def test_drive_all_presets():
    """Drive through all three presets in sequence."""

    workspace = '/home/jasmine/27UCDecision'
    rclpy.init()

    executor = MultiThreadedExecutor()

    # Mock infrastructure nodes (keep alive across presets)
    mock_nav = MockNav2ActionServer()
    signal_pub = SignalPublisher()
    tf_pub = TFPublisher(x=5.0, y=5.0)

    executor.add_node(mock_nav)
    executor.add_node(signal_pub)
    executor.add_node(tf_pub)

    # Run executor in background thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(1.0)  # Let nodes initialize

    presets = [
        ('defense', '基地/前哨侧防守巡航'),
        ('balanced', '中场控制点轮巡 (默认)'),
        ('aggressive', '中场到进攻区前压'),
    ]

    try:
        for i, (preset, desc) in enumerate(presets):
            if i > 0:
                banner(f'=== 切换至下一预设: {preset} ===', '=')

            bt = BTProcess(workspace, preset)
            bt.start()

            # Let the BT load and start patrolling
            phase('等待 BT 加载预设...')
            wait(3.0)
            phase('巡航路线演示: 观察 BT 输出的 SendGoal 坐标序列')

            # Phase A: normal patrol
            phase('[A] 正常巡航 (HP=500, Game=4)')
            signal_pub.set_game_progress(4)
            signal_pub.set_hp(500)
            wait(12.0)

            # Phase B: low HP → supply
            phase('[B] 低血量: HP → 70 (触发补给撤退)')
            signal_pub.set_hp(70)
            wait(8.0)

            # Phase C: partial recovery
            phase('[C] 部分恢复: HP → 200 (仍低于 hp_recover=400，保持补给)')
            signal_pub.set_hp(200)
            wait(5.0)

            # Phase D: full recovery → resume patrol
            phase('[D] 完全恢复: HP → 600 (超过 hp_recover=400，恢复巡航)')
            signal_pub.set_hp(600)
            wait(10.0)

            # Phase E: navigation failure → fallback
            phase('[E] 导航失败: 模拟 Nav2 ABORT')
            mock_nav.set_fail_next(True)
            wait(10.0)

            # Phase F: resume normal
            phase('[F] 恢复正常巡航')
            wait(6.0)

            bt.stop()

            # Brief inter-preset gap
            time.sleep(1.0)

    finally:
        banner('所有预设测试完成', '=')
        executor.shutdown()
        rclpy.shutdown()


# ═══════════════════════════════════════════════════════════════
#  Manual Step-by-Step Interactive Mode
# ═══════════════════════════════════════════════════════════════

def interactive_mode():
    """
    Manually step through scenarios with user prompting between each phase.
    This gives the most visibility into BT log output.
    """
    workspace = '/home/jasmine/27UCDecision'

    rclpy.init()
    executor = MultiThreadedExecutor()

    mock_nav = MockNav2ActionServer()
    signal_pub = SignalPublisher()
    tf_pub = TFPublisher(x=5.0, y=5.0)

    executor.add_node(mock_nav)
    executor.add_node(signal_pub)
    executor.add_node(tf_pub)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(1.0)

    print(Color.BOLD + Color.HEADER + '''
╔══════════════════════════════════════════════════════════════╗
║      RMUC 2026 哨兵决策系统 — 仿真交互测试                     ║
║                                                              ║
║  按回车键进入下一阶段。                                        ║
║  关注 BT 终端窗口中的日志输出。                                 ║
╚══════════════════════════════════════════════════════════════╝
''' + Color.END)

    presets = [
        ('defense',     '基地/前哨防守  cruise'),
        ('balanced',    '中场轮巡 (默认) cruise'),
        ('aggressive',  '进攻区前压      cruise'),
    ]

    for preset, desc in presets:
        banner(f'{preset} — {desc}', '=')
        input(f'{Color.BOLD}按回车启动 {preset} 策略...{Color.END}')

        bt = BTProcess(workspace, preset)
        bt.start()
        wait(2.0)

        # ── Step 1: Game start ──
        input(f'{Color.BOLD}比赛开始 (GameProgress=4), HP=500 — 按回车继续{Color.END}')
        signal_pub.set_game_progress(4)
        signal_pub.set_hp(500)
        wait(12.0)

        # ── Step 2: Low HP ──
        input(f'{Color.BOLD}HP → 70 (低于 hp_retreat=110) — 按回车继续{Color.END}')
        signal_pub.set_hp(70)
        wait(8.0)

        # ── Step 3: Partial recovery ──
        input(f'{Color.BOLD}HP → 200 (低于 hp_recover=400, 继续补给) — 按回车继续{Color.END}')
        signal_pub.set_hp(200)
        wait(5.0)

        # ── Step 4: Full recovery ──
        input(f'{Color.BOLD}HP → 600 (超过 hp_recover=400, 恢复巡航) — 按回车继续{Color.END}')
        signal_pub.set_hp(600)
        wait(8.0)

        # ── Step 5: Navigation failure ──
        input(f'{Color.BOLD}模拟导航失败 (Nav2 ABORT) — 按回车继续{Color.END}')
        mock_nav.set_fail_next(True)
        wait(10.0)

        # ── Step 6: Resume ──
        input(f'{Color.BOLD}恢复正常 — 按回车停止当前预设{Color.END}')
        wait(4.0)

        bt.stop()
        wait(1.0)

    banner('全部测试完成!', '=')
    executor.shutdown()
    rclpy.shutdown()


# ═══════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='RMUC 2026 Decision System Simulation Test')
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='Run in interactive mode (manual step-through)')
    parser.add_argument('--auto', '-a', action='store_true',
                       help='Run in automatic mode (full preset tour)')
    args = parser.parse_args()

    if args.interactive:
        interactive_mode()
    else:
        # Default: automatic mode
        try:
            test_drive_all_presets()
        except KeyboardInterrupt:
            print(f'\n{Color.RED}Interrupted by user{Color.END}')
            rclpy.shutdown()
