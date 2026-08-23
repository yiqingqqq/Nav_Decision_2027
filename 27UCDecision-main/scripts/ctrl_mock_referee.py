#!/usr/bin/env python3
"""可控裁判系统模拟器 — 通过 /tmp/sim_* 触发文件动态改变信号"""
import rclpy, os, time
from rclpy.node import Node
from std_msgs.msg import UInt8, UInt16

class CtrlMockReferee(Node):
    def __init__(self):
        super().__init__('ctrl_mock_referee')
        self.gp = self.create_publisher(UInt8, '/feedback_game_progress', 10)
        self.hp = self.create_publisher(UInt16, '/feedback_robot_hp', 10)
        self._game = 4
        self._hp = 400
        self.create_timer(0.5, self._tick)
        self.create_timer(0.2, self._check_triggers)
        print('[CtrlMock] Started: game=4, hp=400')

    def _check_triggers(self):
        for f, cb in [
            ('/tmp/sim_hp_low',  lambda: setattr(self, '_hp', 70) or print('[HP] 70 (补给)')),
            ('/tmp/sim_hp_mid',  lambda: setattr(self, '_hp', 200) or print('[HP] 200 (滞回)')),
            ('/tmp/sim_hp_high', lambda: setattr(self, '_hp', 600) or print('[HP] 600 (恢复)')),
            ('/tmp/sim_nav_fail', None),
        ]:
            if os.path.exists(f):
                os.unlink(f)
                if cb: cb()

    def _tick(self):
        self.gp.publish(UInt8(data=self._game))
        self.hp.publish(UInt16(data=self._hp))

if __name__ == '__main__':
    rclpy.init(); n = CtrlMockReferee()
    try: rclpy.spin(n)
    except: pass
    finally: rclpy.shutdown()
