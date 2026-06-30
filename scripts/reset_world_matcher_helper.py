#!/usr/bin/env python3
"""reset_world (テレポート) 後に laser_scan_matcher を再起動する

Isaac Sim 側 (launch_isaacsim.py) がロボットを spawn 位置へテレポートすると、
laser_scan_matcher は scan-to-scan ICP のキーフレームが旧位置のまま固定され、
"Error in scan matching" を出し続けて詰まる (失敗時にキーフレームを更新せず、
initialized_ を false に戻す経路も無いため自力復帰できない).

"""

import os
import time

_DEBOUNCE_SEC = 3.0

ROS_VERSION = os.environ.get('HSR_ROS_VERSION', '2').strip()

# ROS2 matcher プロセスを特定する cmdline マッチキー。helper 自身の cmdline
# (reset_world_matcher_helper.py) には含まれないので自己マッチしない。
_ROS2_MATCHER_KEY = 'ros2_laser_scan_matcher'


class _Debouncer:
    def __init__(self):
        self._last = 0.0

    def ready(self):
        now = time.monotonic()
        if now - self._last < _DEBOUNCE_SEC:
            return False
        self._last = now
        return True


def _restart_ros1():
    # ROS1 標準 API でノードを kill する。launch 側の respawn="true" が再起動を担う。
    import rosnode
    print('[reset_world_helper] killing laser_scan_matcher_node (ROS1)', flush=True)
    rosnode.kill_nodes(['/laser_scan_matcher_node'])


def _kill_matcher_processes():
    """matcher プロセスを psutil で kill する (hma_ros2cli の ros2 node kill 同様)。

    1つ以上 kill できたら True。ros2 には rosnode kill 相当が無いため、cmdline に
    _ROS2_MATCHER_KEY を含むプロセスを SIGKILL する。再起動は launch の respawn=True。
    """
    import psutil
    killed = False
    for proc in psutil.process_iter(['pid', 'cmdline', 'name']):
        cmdline = proc.info.get('cmdline') or []
        if any(_ROS2_MATCHER_KEY in part for part in cmdline):
            try:
                proc.kill()
                proc.wait(timeout=3)
            except (psutil.NoSuchProcess, psutil.TimeoutExpired):
                pass
            print('[reset_world_helper] killed laser_scan_matcher (pid=%d)'
                  % proc.pid, flush=True)
            killed = True
    return killed


def _restart_ros2():
    # matcher を kill するだけ。launch 側の respawn=True が同名同パラメータで再起動する。
    print('[reset_world_helper] killing laser_scan_matcher (ROS2)', flush=True)
    for _ in range(3):
        if _kill_matcher_processes():
            return
        time.sleep(0.1)
    print('[reset_world_helper] no laser_scan_matcher process found', flush=True)


def main_ros1():
    import rospy
    from std_msgs.msg import Empty

    rospy.init_node('reset_world_matcher_helper', disable_signals=True)
    deb = _Debouncer()

    def _cb(_msg):
        if deb.ready():
            _restart_ros1()

    rospy.Subscriber('/isaac/reset_world_event', Empty, _cb)
    print('[reset_world_helper] ready (ROS1), waiting for /isaac/reset_world_event', flush=True)
    rospy.spin()


def main_ros2():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Empty

    rclpy.init()
    node = Node('reset_world_matcher_helper')
    deb = _Debouncer()

    def _cb(_msg):
        if deb.ready():
            _restart_ros2()

    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    node.create_subscription(Empty, '/isaac/reset_world_event', _cb, qos)
    node.get_logger().info('ready (ROS2), waiting for /isaac/reset_world_event')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if ROS_VERSION == '1':
        main_ros1()
    else:
        main_ros2()
