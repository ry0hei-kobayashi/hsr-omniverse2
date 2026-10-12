"""ROS 2 アクションサーバ (semu.robotics.ros2_bridge の HSR 向けサブクラス。hsr.py から分離)。"""
# Copyright (c) 2023, Toyota Motor Corporation
# Copyright (c) 2023, MID Academic Promotions, Inc.
# All rights reserved.

import math
import os
import sys
import threading

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.replicator.core as rep
import omni.ui
# from omni.isaac.core.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api import SimulationContext
from isaacsim.core.prims import Articulation
from isaacsim.core.utils import extensions, stage, viewports
from isaacsim.core.utils.prims import set_targets
from isaacsim.core.utils.render_product import create_hydra_texture
from isaacsim.core.utils.stage import get_stage_units
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

# from omni.isaac.sensor import _sensor

import rclpy
import rclpy.node
import rclpy.qos
import rclpy.time
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import (PoseStamped, Quaternion, TransformStamped,
                               Twist, WrenchStamped)
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster
from tmc_control_msgs.action import GripperApplyEffort
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from hsr_omniverse2_sim.robot.hsr.params import (_BASE_GOAL_SETTLE_DURATION, _BASE_GOAL_TIMEOUT, _BASE_GOAL_TOLERANCE, _BASE_GOAL_VELOCITY_TOLERANCE)


from hsr_omniverse2_sim.paths import REPO_ROOT
for _cand in (os.path.join(REPO_ROOT, '3rdparty'), '/app/3rdparty'):
    _semu_src = os.path.join(_cand, 'semu.robotics.ros2_bridge', 'src', 'semu.robotics.ros2_bridge')
    if os.path.isdir(_semu_src):
        sys.path.insert(0, os.path.abspath(_semu_src))
        break
else:
    raise ImportError(
        '3rdparty/semu.robotics.ros2_bridge is missing. Run: git submodule update --init --recursive')
from semu.robotics.ros2_bridge.ros2_bridge import \
    RosControlFollowJointTrajectory as semuInternalState
from semu.robotics.ros2_bridge.ros2_bridge import \
    RosControllerGripperCommand as semuGripperInternalState
class odom_trajectory_action_server(semuInternalState):
    def _init_articulation(self) -> None:
        path = self.articulation_path
        self._articulation = self.dci.get_articulation(path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print(
                '[Warning] FollowJointTrajectory: {} is not an articulation'.format(path))
            return
        for dof_name in ['odom_x', 'odom_y', 'odom_t']:
            self._joints[dof_name] = 0.0
        self._odometry = None
        self._velocity = None
        self._remaining_start_time = None
        self._settle_start_time = None

    def _set_joint_position(self, name: str, target_position: float) -> None:
        if not math.isfinite(float(target_position)):
            print(
                '[trajectory-safety] ignored non-finite base target: '
                '%s=%r' % (name, target_position),
                flush=True,
            )
            return
        self._joints[name] = target_position

    def _get_joint_position(self, name: str) -> float:
        return self._joints[name]

    def _get_time(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def step(self, dt: float) -> None:
        if self._action_goal is not None and self._action_goal_handle is not None:
            if self._odometry is not None and self._action_point_index >= len(
                self._action_goal.trajectory.points
            ):
                errors = (
                    self._get_joint_position('odom_x') - self._odometry.x,
                    self._get_joint_position('odom_y') - self._odometry.y,
                    math.atan2(
                        math.sin(
                            self._get_joint_position('odom_t')
                            - self._odometry.ang
                        ),
                        math.cos(
                            self._get_joint_position('odom_t')
                            - self._odometry.ang
                        ),
                    ),
                )
                max_error = max(abs(error) for error in errors)
                velocities = self._velocity or (float('inf'),) * 3
                max_velocity = max(abs(velocity) for velocity in velocities)
                now = self._get_time()
                if self._remaining_start_time is None:
                    self._remaining_start_time = now
                time_passed = now - self._remaining_start_time
                converged = (
                    max_error <= _BASE_GOAL_TOLERANCE
                    and max_velocity <= _BASE_GOAL_VELOCITY_TOLERANCE
                )
                if converged:
                    if self._settle_start_time is None:
                        self._settle_start_time = now
                    if now - self._settle_start_time < _BASE_GOAL_SETTLE_DURATION:
                        return
                    print(
                        '[base-action] reached: '
                        'target=(%.4f, %.4f, %.4f) '
                        'actual=(%.4f, %.4f, %.4f) '
                        'error=(%.4f, %.4f, %.4f) '
                        'velocity=(%.4f, %.4f, %.4f)'
                        % ((
                                self._get_joint_position('odom_x'),
                                self._get_joint_position('odom_y'),
                                self._get_joint_position('odom_t'),
                                self._odometry.x,
                                self._odometry.y,
                                self._odometry.ang,
                            ) + errors + tuple(velocities)),
                        flush=True,
                    )
                else:
                    self._settle_start_time = None
                if not converged and time_passed < _BASE_GOAL_TIMEOUT:
                    return
                if not converged:
                    result = FollowJointTrajectory.Result()
                    result.error_code = result.GOAL_TOLERANCE_VIOLATED
                    result.error_string = (
                        'base goal tolerance violated: '
                        'dx=%.4f dy=%.4f dyaw=%.4f '
                        'vx=%.4f vy=%.4f vyaw=%.4f'
                        % (errors + tuple(velocities))
                    )
                    print('[base-action] ' + result.error_string, flush=True)
                    handle = self._action_goal_handle
                    self._action_result_message = result
                    if handle is not None:
                        try:
                            handle.abort()
                        except Exception:
                            pass
                    self._action_goal_handle = None
                    self._action_goal = None
                    self._remaining_start_time = None
                    self._settle_start_time = None
                    return
            else:
                self._remaining_start_time = None
                self._settle_start_time = None
        super().step(dt)

    def sample_desired_velocity(self):
        """現在の軌道セグメントの「目標速度」(odom_x/y/t, odom系) を時刻補間で取り出す。

        本物の OmniBaseController と同じく、軌道の各点が持つ velocities を
        フィードフォワード(FF)として使うための источник。位置の差分から速度を
        自作するとセグメント境界でスパイク暴発するため、ここでは軌道が運んでいる
        速度をそのまま読む。velocities が無い軌道なら (0,0,0) を返す(=FFなし)。
        """
        g = self._action_goal
        if g is None:
            return (0.0, 0.0, 0.0)
        pts = g.trajectory.points
        idx = self._action_point_index
        if idx <= 0 or idx >= len(pts):
            return (0.0, 0.0, 0.0)
        prev_p = pts[idx - 1]
        cur_p = pts[idx]
        if not cur_p.velocities or not prev_p.velocities:
            return (0.0, 0.0, 0.0)
        try:
            t0 = self._duration_to_seconds(prev_p.time_from_start)
            t1 = self._duration_to_seconds(cur_p.time_from_start)
            tp = self._get_time() - (self._action_start_time
                                     if self._action_start_time is not None
                                     else self._get_time())
            ratio = 0.0 if t1 <= t0 else max(0.0, min(1.0, (tp - t0) / (t1 - t0)))
        except Exception:
            ratio = 0.0
        out = {'odom_x': 0.0, 'odom_y': 0.0, 'odom_t': 0.0}
        names = g.trajectory.joint_names
        for i, nm in enumerate(names):
            if nm in out and i < len(cur_p.velocities) and i < len(prev_p.velocities):
                out[nm] = (prev_p.velocities[i]
                           + ratio * (cur_p.velocities[i] - prev_p.velocities[i]))
        return (out['odom_x'], out['odom_y'], out['odom_t'])
class arm_trajectory_action_server(semuInternalState):
    controlled_joints = [
        'arm_lift_joint',
        'arm_flex_joint',
        'arm_roll_joint',
        'wrist_flex_joint',
        'wrist_roll_joint',
    ]
    # 位置ドライブなので重力方向の姿勢では必ず「たわみ」が残る。許容値を
    # たわみより小さく取ると毎回「目標未達」で中断してしまうため、実測の
    # たわみ (剛性を上げた後で最大 0.02 rad 程度) に対して余裕を持たせる。
    goal_position_tolerance = {
        'arm_lift_joint': 0.01,
        'arm_flex_joint': 0.06,
        'arm_roll_joint': 0.06,
        'wrist_flex_joint': 0.06,
        'wrist_roll_joint': 0.06,
    }
    goal_velocity_tolerance = {
        'arm_lift_joint': 0.015,
        'arm_flex_joint': 0.08,
        'arm_roll_joint': 0.08,
        'wrist_flex_joint': 0.08,
        'wrist_roll_joint': 0.08,
    }
    goal_settle_duration = 0.25
    goal_timeout = 8.0

    def _set_joint_position(self, name: str, target_position: float) -> None:
        if name == 'arm_lift_joint':
            super()._set_joint_position('torso_lift_joint', target_position / 2.0)
        if name in ['arm_flex_joint', 'arm_lift_joint', 'wrist_flex_joint', 'arm_roll_joint']:
            target_position = -target_position
        super()._set_joint_position(name, target_position)

    def _get_joint_position(self, name: str) -> float:
        v = super()._get_joint_position(name)
        if name in ['arm_flex_joint', 'arm_lift_joint', 'wrist_flex_joint', 'arm_roll_joint']:
            return -v
        return v
class head_trajectory_action_server(semuInternalState):
    controlled_joints = ['head_pan_joint', 'head_tilt_joint']
class gripper_trajectory_action_server(semuInternalState):
    controlled_joints = ['hand_motor_joint']

    def _set_joint_position(self, name: str, target_position: float) -> None:
        if name == 'hand_motor_joint':
            super()._set_joint_position('hand_l_proximal_joint', target_position)
            super()._set_joint_position('hand_l_distal_joint', -target_position)
            super()._set_joint_position('hand_r_proximal_joint', target_position)
            super()._set_joint_position('hand_r_distal_joint', -target_position)
        super()._set_joint_position(name, target_position)
class gripper_command_action_server(semuGripperInternalState):
    def __init__(self, hsr, node=None, _dci=None):
        super().__init__(node, _dci)
        self.gripper_joints_paths = [
            hsr.stage_path + '/hsrb/hand_palm_link/hand_l_proximal_joint',
            hsr.stage_path + '/hsrb/hand_l_mimic_distal_link/hand_l_distal_joint',
            hsr.stage_path + '/hsrb/hand_palm_link/hand_r_proximal_joint',
            hsr.stage_path + '/hsrb/hand_r_mimic_distal_link/hand_r_distal_joint',
        ]

    def _set_joint_position(self, name: str, target_position: float) -> None:
        if name == 'hand_l_distal_joint':
            target_position = -target_position
        if name == 'hand_r_distal_joint':
            target_position = -target_position
        super()._set_joint_position(name, target_position)
class gripper_apply_force_action_server(gripper_command_action_server):
    def __init__(self, hsr, node=None, _dci=None):
        super().__init__(hsr, node, _dci)
        self._action_result_message = GripperApplyEffort.Result()
        self._action_feedback_message = GripperApplyEffort.Feedback()
        # 実機の gripper_controller/grasp, apply_force と同じアクション型で
        # ActionServer を立てる (既定の GripperCommand のままだと型不一致で
        # hsrb_interface クライアントのゴールに応答できず把持が失敗する)。
        self._action_type = GripperApplyEffort
        self._inverse_direction = False
        # 「指が止まった」(stalled) 判定用の連続静止ステップ数カウンタ。
        self._action_stall_steps = 0

    def _get_joint_effort(self, name: str) -> float:
        effort = self.dci.get_dof_state(
            self._joints[name]['dof'], _dynamic_control.STATE_EFFORT
        ).effort
        return effort

    def _get_time(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def step(self, dt: float) -> None:
        if not self._joints:
            self._init_articulation()
            return
        if self._action_goal is not None and self._action_goal_handle is not None:
            # 結果メッセージはローカルで組み立て、succeed()/abort() を呼んだ後に
            # self._action_result_message へ公開する。途中で公開すると、ros2_bridge の
            # _on_execute スレッド (50ms 周期でループ監視) がゴール状態の設定前に
            # return してしまい、rclpy が "Goal state not set, assuming aborted" と
            # して ABORTED を返すレース条件になる (gripper_close が常に失敗する)。
            # 同じ理由で self._action_goal = None も公開より後にする。
            result_message = GripperApplyEffort.Result()
            target_effort = self._action_goal.effort
            if self._inverse_direction:
                target_effort = -target_effort

            # 重要: self._joints は _init_articulation が articulation の全DOF
            # (arm/head/wheel 等すべて) を登録する。ここで全DOFに位置0を出すと、
            # gripper_close のたびに腕が原点(arm_lift=0,arm_flex=0)へ畳まれて、把持点から
            # 離れ常に空振りする (実測で確定: close前 arm=(0.5,-1.0)→close後 (0,0))。
            # → グリッパ(hand_*)の指関節だけに限定する。
            gripper_names = [n for n in self._joints if n.startswith('hand_')]
            self.dci.wake_up_articulation(self._articulation)
            for name in gripper_names:
                if target_effort >= 0.0:
                    self._set_joint_position(name, 0.0)
                else:
                    self._set_joint_position(name, math.pi)

            effort = 0
            effort_reached = True
            for name in gripper_names:
                effort = self._get_joint_effort(name)
                if abs(effort) - abs(target_effort) < 0.0:
                    effort_reached = False
                    break
            if effort_reached:
                result_message.effort = effort
                result_message.stalled = False
                if self._action_goal_handle is not None:
                    self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                self._action_result_message = result_message
                self._action_goal = None
                return

            current_position_sum = 0
            for name in gripper_names:
                position = self._get_joint_position(name)
                current_position_sum += position
            # stalled (指が止まった) 判定。元の閾値 1e-6 は Isaac の物理では
            # 永遠に成立しない (閉じ切った後も指は 5e-4〜1e-3/step 程度のクリープ/
            # 微振動を続けると実測) ため、10 秒タイムアウト→ABORTED になっていた。
            # 実測に基づき閾値 2e-3 とし、連続 10 ステップで stalled 確定とする
            # (閉じ動作中は ~2e-2/step なので明確に区別できる)。
            if self._action_previous_position_sum == float('inf'):
                self._action_stall_steps = 0  # 新しいゴールの開始
            pos_diff = abs(current_position_sum -
                           self._action_previous_position_sum)
            if pos_diff < 2e-3:
                self._action_stall_steps += 1
            else:
                self._action_stall_steps = 0
            if self._action_stall_steps >= 10:
                print('[hsr][gripper] stalled -> succeed (effort=%.3f)' % effort)
                result_message.effort = effort
                result_message.stalled = True
                if self._action_goal_handle is not None:
                    self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                self._action_result_message = result_message
                self._action_goal = None
                return
            self._action_previous_position_sum = current_position_sum

            time_passed = self._get_time() - self._action_start_time
            if time_passed >= self._action_timeout:
                print(
                    '[hsr][gripper] timeout -> abort (stall_steps=%d pos_sum=%.6f time=%.2f)'
                    % (self._action_stall_steps, current_position_sum, time_passed)
                )
                if self._action_goal_handle is not None:
                    self._action_goal_handle.abort()
                    self._action_goal_handle = None
                self._action_result_message = result_message
                self._action_goal = None
