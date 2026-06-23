# Copyright (c) 2023, Toyota Motor Corporation
# Copyright (c) 2023, MID Academic Promotions, Inc.
# All rights reserved.

import math
import os
import sys
import threading

import omni.graph.core as og
import omni.kit.commands
import omni.replicator.core as rep
import omni.ui
# from omni.isaac.core.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from omni.isaac.core import SimulationContext
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils import extensions, nucleus, stage, viewports
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.core.utils.render_product import create_hydra_texture
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

# from omni.isaac.sensor import _sensor

# add by r.kobayashi version selector
ROS_VERSION = os.environ.get('HSR_ROS_VERSION', '2').strip()
if ROS_VERSION not in ('1', '2'):
    raise ValueError(
        f"Unsupported HSR_ROS_VERSION={ROS_VERSION}. Use '1' or '2'.")
is_ros2 = ROS_VERSION == '2'

# is_ros2 = False
if is_ros2:
    import rclpy
    import rclpy.node
    import rclpy.qos
    import rclpy.time
    import tf_transformations
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from geometry_msgs.msg import (PoseStamped, Quaternion, TransformStamped,
                                   Twist, WrenchStamped)
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import Imu, JointState
    from tf2_ros import TransformBroadcaster
    from tmc_control_msgs.action import GripperApplyEffort
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    def quaternion_from_euler(r, p, y):
        return tf_transformations.quaternion_from_euler(r, p, y)

    def euler_from_quaternion(x, y, z, w):
        return tf_transformations.euler_from_quaternion((x, y, z, w))

    extensions.enable_extension('isaacsim.ros2.bridge')

else:
    import actionlib
    import rospy
    import tf.transformations
    from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
    from control_msgs.msg import (FollowJointTrajectoryAction,
                                  FollowJointTrajectoryActionGoal,
                                  FollowJointTrajectoryGoal,
                                  GripperCommandAction,
                                  GripperCommandActionGoal)
    from geometry_msgs.msg import (PoseStamped, Quaternion, TransformStamped,
                                   Twist, WrenchStamped)
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState
    from tmc_control_msgs.msg import (GripperApplyEffortAction,
                                      GripperApplyEffortFeedback,
                                      GripperApplyEffortResult)
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    def quaternion_from_euler(r, p, y):
        return tf.transformations.quaternion_from_euler(r, p, y)

    def euler_from_quaternion(x, y, z, w):
        return tf.transformations.euler_from_quaternion((x, y, z, w))

    extensions.enable_extension('isaacsim.ros1.bridge')
# try:
#    import rospy
#    import tf.transformations
#    import actionlib
#    from geometry_msgs.msg import Twist, PoseStamped, Quaternion, WrenchStamped, TransformStamped
#    from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandActionGoal
#    from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID
#    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#    from sensor_msgs.msg import JointState, Imu
#    from nav_msgs.msg import Odometry
#    from tmc_control_msgs.msg import GripperApplyEffortAction, GripperApplyEffortResult, GripperApplyEffortFeedback
#
#    def quaternion_from_euler(r, p, y):
#        return tf.transformations.quaternion_from_euler(r, p, y)
#
#    def euler_from_quaternion(x, y, z, w):
#        return tf.transformations.euler_from_quaternion((x, y, z, w))
#
#    extensions.enable_extension("isaacsim.ros1.bridge")
# except ImportError:
#
#    is_ros2 = True
#    import rclpy
#    import rclpy.node
#    import rclpy.qos
#    import rclpy.time
#    from geometry_msgs.msg import Twist, PoseStamped, Quaternion, WrenchStamped, TransformStamped
#    from control_msgs.action import FollowJointTrajectory, GripperCommand
#    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#    from sensor_msgs.msg import JointState, Imu
#    from nav_msgs.msg import Odometry
#    from tf2_ros import TransformBroadcaster
#    from tmc_control_msgs.action import GripperApplyEffort
#    from rcl_interfaces.srv import GetParameters
#    from rcl_interfaces.msg import ParameterValue, ParameterType
#    import tf_transformations
#
#    def quaternion_from_euler(r, p, y):
#        return tf_transformations.quaternion_from_euler(r, p, y)
#
#    def euler_from_quaternion(x, y, z, w):
#        return tf_transformations.euler_from_quaternion((x, y, z, w))
#
#    extensions.enable_extension("isaacsim.ros2.bridge")

extensions.enable_extension('isaacsim.sensors.physx')
extensions.enable_extension('isaacsim.util.debug_draw')


def og_ros_node(name):
    if is_ros2:
        return name.replace('.ros1.bridge.', '.ros2.bridge.').replace('.ROS1', '.ROS2')
    return name


if is_ros2 is False:
    print('local module import ')
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '3rdparty'))
    from OgnROS1ActionFollowJointTrajectory import \
        InternalState as semuInternalState
    from OgnROS1ActionGripperCommand import \
        InternalState as semuGripperInternalState

    # extensions.enable_extension("semu.robotics.ros_bridge")
    # from semu.robotics.ros_bridge.ogn.nodes.OgnROS1ActionFollowJointTrajectory import InternalState as semuInternalState
    # from semu.robotics.ros_bridge.ogn.nodes.OgnROS1ActionGripperCommand import InternalState as semuGripperInternalState
else:  # ROS2
    from ros2_bridge import \
        RosControlFollowJointTrajectory as semuInternalState
    from ros2_bridge import \
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
        self._remaining_start_time = None

    def _set_joint_position(self, name: str, target_position: float) -> None:
        self._joints[name] = target_position

    def _get_joint_position(self, name: str) -> float:
        return self._joints[name]

    def _get_time(self) -> float:
        if is_ros2:
            return self._node.get_clock().now().nanoseconds / 1e9
        return rospy.get_time()

    def step(self, dt: float) -> None:
        if self._action_goal is not None and self._action_goal_handle is not None:
            if self._odometry is not None and self._action_point_index >= len(
                self._action_goal.trajectory.points
            ):
                diff = 0.0
                diff += abs(self._get_joint_position('odom_x') -
                            self._odometry.x)
                diff += abs(self._get_joint_position('odom_y') -
                            self._odometry.y)
                diff += abs(self._get_joint_position('odom_t') -
                            self._odometry.ang)
                if self._remaining_start_time is None:
                    self._remaining_start_time = self._get_time()
                time_passed = self._get_time() - self._remaining_start_time
                if diff > 0.001 and time_passed < 5.0:
                    return
            else:
                self._remaining_start_time = None
        super().step(dt)


class arm_trajectory_action_server(semuInternalState):
    controlled_joints = [
        'arm_lift_joint',
        'arm_flex_joint',
        'arm_roll_joint',
        'wrist_flex_joint',
        'wrist_roll_joint',
    ]

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
        if is_ros2 is False:
            super().__init__()
        else:
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
        if is_ros2 is False:
            super().__init__(hsr)
            self._action_result_message = GripperApplyEffortResult()
            self._action_feedback_message = GripperApplyEffortFeedback()
        else:
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
        if is_ros2:
            return self._node.get_clock().now().nanoseconds / 1e9
        return rospy.get_time()

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
            if is_ros2 is False:
                result_message = GripperApplyEffortResult()
            else:
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
                    if is_ros2 is False:
                        self._action_goal_handle.set_succeeded(result_message)
                    else:
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
            pos_diff = abs(current_position_sum - self._action_previous_position_sum)
            if pos_diff < 2e-3:
                self._action_stall_steps += 1
            else:
                self._action_stall_steps = 0
            if self._action_stall_steps >= 10:
                print('[hsr][gripper] stalled -> succeed (effort=%.3f)' % effort)
                result_message.effort = effort
                result_message.stalled = True
                if self._action_goal_handle is not None:
                    if is_ros2 is False:
                        self._action_goal_handle.set_aborted(result_message)
                    else:
                        # stalled = 指が止まった状態。物を掴んで止まるのは正常な把持
                        # 完了なので、実機の gripper controller と同様 succeed を返す
                        # (結果の stalled=True で状態は伝わる)。abort のままだと
                        # hsrb_interface の apply_force が
                        # "Failed to apply force state 6" 例外を投げ、空把持でも
                        # 物を掴んだときでも gripper_close が常に失敗扱いになる。
                        self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                self._action_result_message = result_message
                self._action_goal = None
                return
            self._action_previous_position_sum = current_position_sum

            time_passed = self._get_time() - self._action_start_time
            if time_passed >= self._action_timeout:
                print('[hsr][gripper] timeout -> abort (stall_steps=%d pos_sum=%.6f time=%.2f)'
                      % (self._action_stall_steps, current_position_sum, time_passed))
                if self._action_goal_handle is not None:
                    if is_ros2 is False:
                        self._action_goal_handle.set_aborted()
                    else:
                        self._action_goal_handle.abort()
                    self._action_goal_handle = None
                self._action_result_message = result_message
                self._action_goal = None


# 案A(アタッチ把持)を使うか。recol(衝突)と併用して確実な保持・綺麗なリリースにする。
_ATTACH_GRASP_ENABLED = True


# --- 案A(アタッチ把持)用クォータニオン補助 (x,y,z,w) ---
def _q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _q_rot(q, v):
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx))


class hsr_config:
    def __init__(self) -> None:
        self.use_ros = True


wheel_separation = 0.266
wheel_radius = 0.04
wheel_offset = 0.11


class BaseOdometry:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.ang = 0.0


class JointSpace:
    def __init__(self) -> None:
        self.vel_wheel_l = 0.0
        self.vel_wheel_r = 0.0
        self.vel_steer = 0.0


class CartSpace:
    def __init__(self) -> None:
        self.dot_x = 0.0
        self.dot_y = 0.0
        self.dot_r = 0.0


class VehicleState:
    def __init__(self) -> None:
        self.steer_angle = 0.0


class VehicleDynamics:
    """
    Dynamics of offset diff drive vehicle
    Equations are from the paper written by Masayoshi Wada etal.
    https://www.jstage.jst.go.jp/article/jrsj1983/18/8/18_8_1166/_pdf
    """

    def __init__(self, wheel_radius: float, wheel_separation: float, wheel_offset: float) -> None:
        self._wheel_radius = wheel_radius
        self._wheel_separation = wheel_separation
        self._wheel_offset = wheel_offset

    def forward(self, joint_space: JointSpace, state: VehicleState) -> CartSpace:
        cos_s = math.cos(state.steer_angle)
        sin_s = math.sin(state.steer_angle)
        output = CartSpace()
        output.dot_x = (
            self._wheel_radius / 2.0 * cos_s
            - self._wheel_radius * self._wheel_offset / self._wheel_separation * sin_s
        ) * joint_space.vel_wheel_r + (
            self._wheel_radius / 2.0 * cos_s
            + self._wheel_radius * self._wheel_offset / self._wheel_separation * sin_s
        ) * joint_space.vel_wheel_l
        output.dot_y = (
            self._wheel_radius / 2.0 * sin_s
            + self._wheel_radius * self._wheel_offset / self._wheel_separation * cos_s
        ) * joint_space.vel_wheel_r + (
            self._wheel_radius / 2.0 * sin_s
            - self._wheel_radius * self._wheel_offset / self._wheel_separation * cos_s
        ) * joint_space.vel_wheel_l
        output.dot_r = (
            self._wheel_radius / self._wheel_separation * joint_space.vel_wheel_r
            - self._wheel_radius / self._wheel_separation * joint_space.vel_wheel_l
            - joint_space.vel_steer
        )
        return output

    def inverse(self, cart_space: CartSpace, state: VehicleState) -> JointSpace:
        cos_s = math.cos(state.steer_angle)
        sin_s = math.sin(state.steer_angle)
        output = JointSpace()
        output.vel_wheel_r = (
            cos_s / self._wheel_radius
            - self._wheel_separation * sin_s / 2.0 /
            self._wheel_radius / self._wheel_offset
        ) * cart_space.dot_x + (
            sin_s / self._wheel_radius
            + self._wheel_separation * cos_s / 2.0 /
            self._wheel_radius / self._wheel_offset
        ) * cart_space.dot_y
        output.vel_wheel_l = (
            cos_s / self._wheel_radius
            + self._wheel_separation * sin_s / 2.0 /
            self._wheel_radius / self._wheel_offset
        ) * cart_space.dot_x + (
            sin_s / self._wheel_radius
            - self._wheel_separation * cos_s / 2.0 /
            self._wheel_radius / self._wheel_offset
        ) * cart_space.dot_y
        output.vel_steer = (
            -sin_s / self._wheel_offset * cart_space.dot_x
            + cos_s / self._wheel_offset * cart_space.dot_y
            - cart_space.dot_r
        )
        return output


class WheelOdometry:
    def __init__(self) -> None:
        self.pose = BaseOdometry()

    def integrate(self, cart_velocity: CartSpace, dt: float):
        diff_r = cart_velocity.dot_r * dt
        cosr = math.cos(self.pose.ang + 0.5 * diff_r)
        sinr = math.sin(self.pose.ang + 0.5 * diff_r)
        world_dot_x = cart_velocity.dot_x * cosr - cart_velocity.dot_y * sinr
        world_dot_y = cart_velocity.dot_x * sinr + cart_velocity.dot_y * cosr
        if dt > 0.0:
            self.pose.x += world_dot_x * dt
            self.pose.y += world_dot_y * dt
            self.pose.ang += diff_r
        return world_dot_x, world_dot_y

    def set_pose(self, x: float, y: float, ang: float) -> None:
        self.pose.x = x
        self.pose.y = y
        self.pose.ang = ang


class hsr:
    def __init__(self, prefix='/hsrb', stage_path='/World', config=None) -> None:
        global is_ros2

        if config is None:
            config = hsr_config()

        # if is_ros2:
        #    rclpy.init()
        #    self.ros2node = rclpy.node.Node("isaac_sim_hsr")
        #    self.create_subscriber = lambda t, d, c: self.ros2node.create_subscription(d, t, c, qos_profile=rclpy.qos.qos_profile_sensor_data)
        #    self.create_publisher = lambda t, d: self.ros2node.create_publisher(d, t, qos_profile=rclpy.qos.qos_profile_sensor_data)
        #    self.create_publisher_reliable = lambda t, d: self.ros2node.create_publisher(d, t, qos_profile=rclpy.qos.qos_profile_system_default)
        #    self.get_ros_time = lambda t: rclpy.time.Time(seconds=t).to_msg()
        #    self.tf_broadcaster = TransformBroadcaster(self.ros2node)
        #    self._create_controller_parameter_services()
        #    executor = rclpy.executors.MultiThreadedExecutor()
        #    executor.add_node(self.ros2node)
        #    threading.Thread(target=executor.spin).start()
        # else:
        #    # add by r.kobayashi
        #    try:
        #        rospy.init_node("isaac_sim_hsr", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
        #    except rospy.exception.ROSException:
        #        pass

        #    self.create_subscriber = lambda t, d, c: rospy.Subscriber(t, d, c)
        #    self.create_publisher = lambda t, d: rospy.Publisher(t, d, queue_size=5)
        #    self.create_publisher_reliable = lambda t, d: rospy.Publisher(t, d, queue_size=5)
        #    self.get_ros_time = lambda t: rospy.Time(t)
        if is_ros2:
            if not rclpy.ok():
                rclpy.init()
            self.ros2node = rclpy.node.Node('isaac_sim_hsr')
            self.create_subscriber = lambda t, d, c: self.ros2node.create_subscription(
                d, t, c, qos_profile=rclpy.qos.qos_profile_sensor_data
            )
            self.create_publisher = lambda t, d: self.ros2node.create_publisher(
                d, t, qos_profile=rclpy.qos.qos_profile_sensor_data
            )
            self.create_publisher_reliable = lambda t, d: self.ros2node.create_publisher(
                d, t, qos_profile=rclpy.qos.qos_profile_system_default
            )
            self.get_ros_time = lambda t: rclpy.time.Time(seconds=t).to_msg()
            self.tf_broadcaster = TransformBroadcaster(self.ros2node)
            self._create_controller_parameter_services()

            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(self.ros2node)
            self._executor = executor
            self._executor_thread = threading.Thread(
                target=executor.spin, daemon=True)
            self._executor_thread.start()
        else:
            try:
                rospy.init_node(
                    'isaac_sim_hsr', anonymous=True, disable_signals=True, log_level=rospy.ERROR
                )
            except rospy.exceptions.ROSException:
                pass

            self.create_subscriber = lambda t, d, c: rospy.Subscriber(t, d, c)
            self.create_publisher = lambda t, d: rospy.Publisher(
                t, d, queue_size=5)
            self.create_publisher_reliable = lambda t, d: rospy.Publisher(
                t, d, queue_size=5)
            self.get_ros_time = lambda t: rospy.Time.from_sec(t)

        self.prefix = prefix
        self.stage_path = stage_path
        self.simulation_context = None
        self.art = None
        # HSR モデル(usd)の場所を、配置に依らず見つける。
        #   - 焼き込みフラット配置: /app/hsr.py    → /app/usd/hsrb/hsrb4s.usd
        #   - リポジトリ配置:       /app/scripts/hsr.py → /app/usd/hsrb/hsrb4s.usd
        #     (usd は scripts の隣ではなくリポジトリ直下にあるため '..' を見る)
        _here = os.path.dirname(os.path.abspath(__file__))
        _hsr_usd_candidates = [
            os.path.join(_here, 'usd', 'hsrb', 'hsrb4s.usd'),
            os.path.join(_here, '..', 'usd', 'hsrb', 'hsrb4s.usd'),
            '/app/usd/hsrb/hsrb4s.usd',
        ]
        _hsr_usd = next(
            (p for p in _hsr_usd_candidates if os.path.exists(p)),
            _hsr_usd_candidates[0],
        )
        self.hsr = stage.add_reference_to_stage(
            _hsr_usd,
            self.stage_path + self.prefix,
        )
        self.set_base_joint_and_material()

        self.create_cameras()
        self.create_lidar()
        self.create_imu()

        if is_ros2:
            self.laserscan_pose_sub = self.create_subscriber(
                '/laser_odom', Odometry, self.on_laserscan_odom
            )
            self.base_odom_pub = self.create_publisher_reliable(
                '/omni_base_controller/wheel_odom', Odometry
            )
        else:
            self.laserscan_pose_sub = self.create_subscriber(
                self.prefix + '/laser_scan_matcher/pose', PoseStamped, self.on_laserscan_pose
            )
            self.laser_odom_pub = self.create_publisher(
                self.prefix + '/laser_odom', Odometry)
            self.base_odom_pub = self.create_publisher('/odom', Odometry)

        self.robots = ArticulationView(
            prim_paths_expr=self.stage_path + self.prefix, name='hsr_view'
        )
        # wrench は RELIABLE で出す。実機の /hsrb/wrist_wrench/* は RELIABLE で、
        # skill の is_hand_collision は rclpy.wait_for_message を既定QoS(RELIABLE)で
        # 購読する。BEST_EFFORT で出すと QoS 不一致でメッセージが届かず
        # wait_for_message が永久ブロックする (実測)。
        self.ft_sensor_pub = self.create_publisher_reliable(
            self.prefix + '/wrist_wrench/raw', WrenchStamped)
        # 重力補正済み wrench。実機は compensated を出すので skill は
        # こちらを優先購読する。EMA で重力(=ゆっくり変化)を差し引き、
        # 接触の過渡だけ残す。
        self.ft_sensor_comp_pub = self.create_publisher_reliable(
            self.prefix + '/wrist_wrench/compensated', WrenchStamped)
        self._wrench_bias = [0.0] * 6
        self._wrench_bias_inited = False

        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        self.cmd_vel_msg = None
        self.last_cmd_vel_time = 0.0
        self.create_subscriber(
            '/omni_base_controller/cmd_vel' if is_ros2 else self.prefix + '/command_velocity',
            Twist,
            self.on_cmd_vel,
        )
        self.vehicle_dynamics = VehicleDynamics(
            wheel_radius, wheel_separation, wheel_offset)
        self.odometry_estimator = WheelOdometry()
        self.vel_limit_steer_ = 8.0
        self.vel_limit_wheel_ = 8.0

        self.joint_state_pub = self.create_publisher_reliable(
            '/joint_states' if is_ros2 else self.prefix + '/joint_states', JointState
        )

        if is_ros2 is False:

            def init_action_server(srv, name, msg):
                srv.articulation_path = self.stage_path + self.prefix
                srv.usd_context = omni.usd.get_context()
                srv.dci = self.dc
                action_topic_name = self.prefix + '/' + name
                srv.action_server = actionlib.ActionServer(
                    action_topic_name,
                    msg,
                    goal_cb=srv.on_goal,
                    cancel_cb=srv.on_cancel,
                    auto_start=False,
                )
                srv.action_server.start()
                if msg == FollowJointTrajectoryAction:
                    srv.action_client = actionlib.SimpleActionClient(
                        action_topic_name, msg)

                    def command_topic_callback(msg, args):
                        srv = args[0]
                        goal = FollowJointTrajectoryGoal(trajectory=msg)
                        if srv._action_goal is not None:
                            for name in goal.trajectory.joint_names:
                                if name not in self._joints:
                                    print(
                                        "[Warning][semu.robotics.ros_bridge] ROS1 FollowJointTrajectory: joints don't match ({} not in {})".format(
                                            name, list(self._joints.keys())
                                        )
                                    )
                                    return
                            if goal.trajectory.points[0].time_from_start.to_sec():
                                initial_point = JointTrajectoryPoint(
                                    positions=[
                                        srv._get_joint_position(name)
                                        for name in goal.trajectory.joint_names
                                    ],
                                    time_from_start=rospy.Duration(),
                                )
                                goal.trajectory.points.insert(0, initial_point)
                            srv._action_goal = goal
                            srv._action_point_index = 1
                            srv._action_start_time = rospy.get_time()
                            srv._action_feedback_message.joint_names = list(
                                goal.trajectory.joint_names
                            )
                        else:
                            srv.action_client.send_goal(goal)

                    srv.topic_interface = rospy.Subscriber(
                        action_topic_name.replace(
                            '/follow_joint_trajectory', '/command'),
                        JointTrajectory,
                        command_topic_callback,
                        (srv,),
                    )
                srv.initialized = True

            self.arm_trajectory_action_server = arm_trajectory_action_server()
            init_action_server(
                self.arm_trajectory_action_server,
                'arm_trajectory_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction,
            )

            self.head_trajectory_action_server = head_trajectory_action_server()
            init_action_server(
                self.head_trajectory_action_server,
                'head_trajectory_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction,
            )

            self.odom_trajectory_action_server = odom_trajectory_action_server()
            init_action_server(
                self.odom_trajectory_action_server,
                'omni_base_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction,
            )

            self.gripper_trajectory_action_server = gripper_trajectory_action_server()
            init_action_server(
                self.gripper_trajectory_action_server,
                'gripper_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction,
            )

            self.gripper_apply_force_action_server = gripper_apply_force_action_server(
                self)
            init_action_server(
                self.gripper_apply_force_action_server,
                'gripper_controller/apply_force',
                GripperApplyEffortAction,
            )

            self.gripper_command_action_server = gripper_apply_force_action_server(
                self)
            self.gripper_command_action_server._inverse_direction = True
            init_action_server(
                self.gripper_command_action_server,
                'gripper_controller/grasp',
                GripperApplyEffortAction,
            )
        else:

            def init_action_server(srv, name, msg):
                articulation_namespace = self.stage_path + self.prefix
                action_topic_name = '/' + name
                srv.start(articulation_namespace, action_topic_name)

            self.arm_trajectory_action_server = arm_trajectory_action_server(
                self.ros2node, self.dc)
            init_action_server(
                self.arm_trajectory_action_server,
                'arm_trajectory_controller/follow_joint_trajectory',
                FollowJointTrajectory,
            )

            self.head_trajectory_action_server = head_trajectory_action_server(
                self.ros2node, self.dc
            )
            init_action_server(
                self.head_trajectory_action_server,
                'head_trajectory_controller/follow_joint_trajectory',
                FollowJointTrajectory,
            )

            self.odom_trajectory_action_server = odom_trajectory_action_server(
                self.ros2node, self.dc
            )
            init_action_server(
                self.odom_trajectory_action_server,
                'omni_base_controller/follow_joint_trajectory',
                FollowJointTrajectory,
            )

            self.gripper_trajectory_action_server = gripper_trajectory_action_server(
                self.ros2node, self.dc
            )
            init_action_server(
                self.gripper_trajectory_action_server,
                'gripper_controller/follow_joint_trajectory',
                FollowJointTrajectory,
            )

            self.gripper_apply_force_action_server = gripper_apply_force_action_server(
                self, self.ros2node, self.dc
            )
            init_action_server(
                self.gripper_apply_force_action_server,
                'gripper_controller/apply_force',
                GripperApplyEffort,
            )

            self.gripper_command_action_server = gripper_apply_force_action_server(
                self, self.ros2node, self.dc
            )
            self.gripper_command_action_server._inverse_direction = True
            init_action_server(
                self.gripper_command_action_server, 'gripper_controller/grasp', GripperApplyEffort
            )

    def create_cameras(self) -> None:
        topic_prefix = '' if is_ros2 else self.prefix

        l_camera_prim = UsdGeom.Camera(
            omni.usd
            .get_context()
            .get_stage()
            .DefinePrim(
                self.stage_path + self.prefix + '/head_l_stereo_camera_link/Camera', 'Camera'
            )
        )
        xform_api = UsdGeom.XformCommonAPI(l_camera_prim)
        xform_api.SetRotate(
            (180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        l_camera_prim.GetHorizontalApertureAttr().Set(1280 * 0.003)
        l_camera_prim.GetVerticalApertureAttr().Set(960 * 0.003)
        l_camera_prim.GetProjectionAttr().Set('perspective')
        l_camera_prim.GetFocalLengthAttr().Set(968.770306867 * 0.003)
        l_camera_prim.GetFocusDistanceAttr().Set(400)

        r_camera_prim = UsdGeom.Camera(
            omni.usd
            .get_context()
            .get_stage()
            .DefinePrim(
                self.stage_path + self.prefix + '/head_r_stereo_camera_link/Camera', 'Camera'
            )
        )
        xform_api = UsdGeom.XformCommonAPI(r_camera_prim)
        xform_api.SetRotate(
            (180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        r_camera_prim.GetHorizontalApertureAttr().Set(1280 * 0.003)
        r_camera_prim.GetVerticalApertureAttr().Set(960 * 0.003)
        r_camera_prim.GetProjectionAttr().Set('perspective')
        r_camera_prim.GetFocalLengthAttr().Set(968.770306867 * 0.003)
        r_camera_prim.GetFocusDistanceAttr().Set(400)

        rgbd_camera_prim = UsdGeom.Camera(
            omni.usd
            .get_context()
            .get_stage()
            .DefinePrim(self.stage_path + self.prefix + '/head_rgbd_sensor_link/Camera', 'Camera')
        )
        xform_api = UsdGeom.XformCommonAPI(rgbd_camera_prim)
        xform_api.SetRotate(
            (180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        rgbd_camera_prim.GetHorizontalApertureAttr().Set(640 * 0.003)
        rgbd_camera_prim.GetVerticalApertureAttr().Set(480 * 0.003)
        rgbd_camera_prim.GetProjectionAttr().Set('perspective')
        rgbd_camera_prim.GetFocalLengthAttr().Set(554.382712823 * 0.003)
        rgbd_camera_prim.GetFocusDistanceAttr().Set(400)

        hand_camera_prim = UsdGeom.Camera(
            omni.usd
            .get_context()
            .get_stage()
            .DefinePrim(self.stage_path + self.prefix + '/hand_camera_frame/Camera', 'Camera')
        )
        xform_api = UsdGeom.XformCommonAPI(hand_camera_prim)
        xform_api.SetRotate(
            (180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        hand_camera_prim.GetHorizontalApertureAttr().Set(640 * 0.003)
        hand_camera_prim.GetVerticalApertureAttr().Set(480 * 0.003)
        hand_camera_prim.GetProjectionAttr().Set('perspective')
        hand_camera_prim.GetFocalLengthAttr().Set(205.469637099 * 0.003)
        hand_camera_prim.GetFocusDistanceAttr().Set(400)

        try:
            og.Controller.edit(
                {'graph_path': '/ros_controllers', 'evaluator_name': 'execution'},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ('OnImpulseEvent', 'omni.graph.action.OnImpulseEvent'),
                        ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                        ('PublishClock', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1PublishClock')),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ('OnImpulseEvent.outputs:execOut',
                         'PublishClock.inputs:execIn'),
                        ('ReadSimTime.outputs:simulationTime',
                         'PublishClock.inputs:timeStamp'),
                    ],
                    og.Controller.Keys.SET_VALUES: [],
                },
            )

            (self.ros_camera_graph_l, _, _, _) = og.Controller.edit(
                {
                    'graph_path': '/head_l_camera',
                    'evaluator_name': 'push',
                    'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ('OnTick', 'omni.graph.action.OnTick'),
                        ('createRenderProduct',
                         'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                        ('cameraHelperRgb', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                        ('cameraHelperInfo', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ('OnTick.outputs:tick', 'createRenderProduct.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperRgb.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperInfo.inputs:execIn'),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperRgb.inputs:renderProductPath',
                        ),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperInfo.inputs:renderProductPath',
                        ),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ('createRenderProduct.inputs:width', 1280),
                        ('createRenderProduct.inputs:height', 960),
                        ('cameraHelperRgb.inputs:frameId',
                         'head_l_stereo_camera_frame'),
                        (
                            'cameraHelperRgb.inputs:topicName',
                            topic_prefix + '/head_l_stereo_camera/image_rect_color',
                        ),
                        ('cameraHelperRgb.inputs:type', 'rgb'),
                        ('cameraHelperInfo.inputs:frameId',
                         'head_l_stereo_camera_frame'),
                        (
                            'cameraHelperInfo.inputs:topicName',
                            topic_prefix + '/head_l_stereo_camera/camera_info',
                        ),
                        ('cameraHelperInfo.inputs:type', 'camera_info'),
                    ],
                },
            )

            (self.ros_camera_graph_r, _, _, _) = og.Controller.edit(
                {
                    'graph_path': '/head_r_camera',
                    'evaluator_name': 'push',
                    'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ('OnTick', 'omni.graph.action.OnTick'),
                        ('createRenderProduct',
                         'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                        ('cameraHelperRgb', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                        ('cameraHelperInfo', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ('OnTick.outputs:tick', 'createRenderProduct.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperRgb.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperInfo.inputs:execIn'),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperRgb.inputs:renderProductPath',
                        ),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperInfo.inputs:renderProductPath',
                        ),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ('createRenderProduct.inputs:width', 1280),
                        ('createRenderProduct.inputs:height', 960),
                        ('cameraHelperRgb.inputs:frameId',
                         'head_r_stereo_camera_frame'),
                        (
                            'cameraHelperRgb.inputs:topicName',
                            topic_prefix + '/head_r_stereo_camera/image_rect_color',
                        ),
                        ('cameraHelperRgb.inputs:type', 'rgb'),
                        ('cameraHelperInfo.inputs:frameId',
                         'head_r_stereo_camera_frame'),
                        (
                            'cameraHelperInfo.inputs:topicName',
                            topic_prefix + '/head_r_stereo_camera/camera_info',
                        ),
                        ('cameraHelperInfo.inputs:type', 'camera_info'),
                    ],
                },
            )

            (self.ros_camera_graph_rgbd, _, _, _) = og.Controller.edit(
                {
                    'graph_path': '/head_rgbd_camera',
                    'evaluator_name': 'push',
                    'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ('OnTick', 'omni.graph.action.OnTick'),
                        ('createRenderProduct',
                         'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                        ('cameraHelperRgb', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                        ('cameraHelperInfo', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                        ('cameraHelperDepth', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                        (
                            'cameraHelperDepthInfo',
                            og_ros_node(
                                'isaacsim.ros1.bridge.ROS1CameraHelper'),
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ('OnTick.outputs:tick', 'createRenderProduct.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperRgb.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperInfo.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperDepth.inputs:execIn'),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperRgb.inputs:renderProductPath',
                        ),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperInfo.inputs:renderProductPath',
                        ),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperDepth.inputs:renderProductPath',
                        ),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperDepthInfo.inputs:renderProductPath',
                        ),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ('createRenderProduct.inputs:width', 640),
                        ('createRenderProduct.inputs:height', 480),
                        ('cameraHelperRgb.inputs:frameId',
                         'head_rgbd_sensor_rgb_frame'),
                        (
                            'cameraHelperRgb.inputs:topicName',
                            topic_prefix + '/head_rgbd_sensor/rgb/image_rect_color',
                        ),
                        ('cameraHelperRgb.inputs:type', 'rgb'),
                        ('cameraHelperInfo.inputs:frameId',
                         'head_rgbd_sensor_rgb_frame'),
                        (
                            'cameraHelperInfo.inputs:topicName',
                            topic_prefix + '/head_rgbd_sensor/rgb/camera_info',
                        ),
                        ('cameraHelperInfo.inputs:type', 'camera_info'),
                        ('cameraHelperDepth.inputs:frameId',
                         'head_rgbd_sensor_rgb_frame'),
                        (
                            'cameraHelperDepth.inputs:topicName',
                            topic_prefix + '/head_rgbd_sensor/depth_registered/image_rect_raw',
                        ),
                        ('cameraHelperDepth.inputs:type', 'depth'),
                        # Sensor Data QoS (BEST_EFFORT) so the depth image
                        # propagates over CycloneDDS PC unicast to remote
                        # subscribers (e.g. pumas_navigation on a separate PC).
                        # Keys must use camelCase (keepLast/bestEffort) per
                        # Isaac Sim 4.5 OgnROS2QoSProfile schema.
                        (
                            'cameraHelperDepth.inputs:qosProfile',
                            '{"history":"keepLast","depth":5,"reliability":"bestEffort","durability":"volatile","deadline":0.0,"lifespan":0.0,"liveliness":"systemDefault","leaseDuration":0.0}',
                        ),
                        ('cameraHelperDepthInfo.inputs:frameId',
                         'head_rgbd_sensor_rgb_frame'),
                        (
                            'cameraHelperDepthInfo.inputs:topicName',
                            topic_prefix + '/head_rgbd_sensor/depth_registered/camera_info',
                        ),
                        ('cameraHelperDepthInfo.inputs:type', 'camera_info'),
                        # camera_info stays RELIABLE (default) — small payload,
                        # and depth_image_proc::PointCloudXyzrgbNode subscribes
                        # via image_transport/message_filters which does NOT
                        # honor qos_overrides parameters. Keeping RELIABLE
                        # avoids the QoS mismatch that blocks pointcloud output.
                    ],
                },
            )

            (self.ros_camera_graph_hand, _, _, _) = og.Controller.edit(
                {
                    'graph_path': '/hand_camera',
                    'evaluator_name': 'push',
                    'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ('OnTick', 'omni.graph.action.OnTick'),
                        ('createRenderProduct',
                         'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                        ('cameraHelperRgb', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                        ('cameraHelperInfo', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ('OnTick.outputs:tick', 'createRenderProduct.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperRgb.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperInfo.inputs:execIn'),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperRgb.inputs:renderProductPath',
                        ),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperInfo.inputs:renderProductPath',
                        ),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ('createRenderProduct.inputs:width', 640),
                        ('createRenderProduct.inputs:height', 480),
                        ('cameraHelperRgb.inputs:frameId', 'hand_camera_frame'),
                        (
                            'cameraHelperRgb.inputs:topicName',
                            topic_prefix + '/hand_camera/image_raw',
                        ),
                        ('cameraHelperRgb.inputs:type', 'rgb'),
                        ('cameraHelperInfo.inputs:frameId', 'hand_camera_frame'),
                        (
                            'cameraHelperInfo.inputs:topicName',
                            topic_prefix + '/hand_camera/camera_info',
                        ),
                        ('cameraHelperInfo.inputs:type', 'camera_info'),
                    ],
                },
            )
        except Exception as e:
            raise e

        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(
                '/head_l_camera/createRenderProduct'),
            attribute='inputs:cameraPrim',
            target_prim_paths=[self.stage_path + self.prefix +
                               '/head_l_stereo_camera_link/Camera'],
        )

        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(
                '/head_r_camera/createRenderProduct'),
            attribute='inputs:cameraPrim',
            target_prim_paths=[self.stage_path + self.prefix +
                               '/head_r_stereo_camera_link/Camera'],
        )

        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(
                '/head_rgbd_camera/createRenderProduct'),
            attribute='inputs:cameraPrim',
            target_prim_paths=[self.stage_path +
                               self.prefix + '/head_rgbd_sensor_link/Camera'],
        )

        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(
                '/hand_camera/createRenderProduct'),
            attribute='inputs:cameraPrim',
            target_prim_paths=[self.stage_path +
                               self.prefix + '/hand_camera_frame/Camera'],
        )

        og.Controller.evaluate_sync(self.ros_camera_graph_l)
        og.Controller.evaluate_sync(self.ros_camera_graph_r)
        og.Controller.evaluate_sync(self.ros_camera_graph_rgbd)
        og.Controller.evaluate_sync(self.ros_camera_graph_hand)

    def create_lidar_rtx(self) -> None:
        lidar_config = 'Example_Rotary'
        _, sensor = omni.kit.commands.execute(
            'IsaacSensorCreateRtxLidar',
            path=self.stage_path + self.prefix + '/base_range_sensor_link/Lidar',
            parent=None,
            config=lidar_config,
        )
        _, render_product_path = create_hydra_texture(
            [1, 1], sensor.GetPath().pathString)
        writer = rep.writers.get('RtxLidar' + 'DebugDrawPointCloud')
        writer.attach([render_product_path])
        writer = rep.writers.get('RtxLidar' + 'ROS1PublishPointCloud')
        writer.attach([render_product_path])

    def create_imu(self) -> None:
        _, sensor = omni.kit.commands.execute(
            'IsaacSensorCreateImuSensor',
            path=self.stage_path + self.prefix + '/base_imu_frame/Imu_Sensor',
            parent=None,
            sensor_period=-1,
        )
        (self.ros_imu, _, _, _) = og.Controller.edit(
            {
                'graph_path': self.stage_path + self.prefix + '/imu_sensor',
                'evaluator_name': 'execution',
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ('OnTick', 'omni.graph.action.OnPlaybackTick'),
                    ('readSimulationTime',
                     'isaacsim.core.nodes.IsaacReadSimulationTime'),
                    ('readImu', 'isaacsim.sensors.physics.IsaacReadIMU'),
                    ('publishImu', og_ros_node('isaacsim.ros1.bridge.ROS1PublishImu')),
                ],
                og.Controller.Keys.CONNECT: [
                    ('OnTick.outputs:tick', 'readImu.inputs:execIn'),
                    ('readImu.outputs:execOut', 'publishImu.inputs:execIn'),
                    ('readSimulationTime.outputs:simulationTime',
                     'publishImu.inputs:timeStamp'),
                    ('readImu.outputs:linAcc',
                     'publishImu.inputs:linearAcceleration'),
                    ('readImu.outputs:angVel', 'publishImu.inputs:angularVelocity'),
                    ('readImu.outputs:orientation',
                     'publishImu.inputs:orientation'),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ('publishImu.inputs:frameId', 'base_imu_frame'),
                    ('publishImu.inputs:topicName',
                     self.prefix + '/base_imu/data'),
                ],
            },
        )
        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(
                self.stage_path + self.prefix + '/imu_sensor/readImu'
            ),
            attribute='inputs:imuPrim',
            target_prim_paths=[self.stage_path +
                               self.prefix + '/base_imu_frame/Imu_Sensor'],
        )

    def create_lidar(self) -> None:
        _, sensor = omni.kit.commands.execute(
            'RangeSensorCreateLidar',
            path=self.stage_path + self.prefix + '/base_range_sensor_link/Lidar',
            parent=None,
            min_range=0.3,
            max_range=60.0,
            draw_points=False,
            draw_lines=True,
            horizontal_fov=240.0,
            horizontal_resolution=0.25,
            rotation_rate=30,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False,
        )
        (self.ros_lidar, _, _, _) = og.Controller.edit(
            {
                'graph_path': '/lidar_sensor',
                'evaluator_name': 'execution',
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ('OnTick', 'omni.graph.action.OnPlaybackTick'),
                    ('readSimulationTime',
                     'isaacsim.core.nodes.IsaacReadSimulationTime'),
                    ('readLidarBeams', 'isaacsim.sensors.physx.IsaacReadLidarBeams'),
                    ('publishLaserScan', og_ros_node(
                        'isaacsim.ros1.bridge.ROS1PublishLaserScan')),
                ],
                og.Controller.Keys.CONNECT: [
                    ('OnTick.outputs:tick', 'readLidarBeams.inputs:execIn'),
                    ('readLidarBeams.outputs:execOut',
                     'publishLaserScan.inputs:execIn'),
                    (
                        'readSimulationTime.outputs:simulationTime',
                        'publishLaserScan.inputs:timeStamp',
                    ),
                    (
                        'readLidarBeams.outputs:horizontalFov',
                        'publishLaserScan.inputs:horizontalFov',
                    ),
                    (
                        'readLidarBeams.outputs:horizontalResolution',
                        'publishLaserScan.inputs:horizontalResolution',
                    ),
                    ('readLidarBeams.outputs:depthRange',
                     'publishLaserScan.inputs:depthRange'),
                    ('readLidarBeams.outputs:rotationRate',
                     'publishLaserScan.inputs:rotationRate'),
                    (
                        'readLidarBeams.outputs:linearDepthData',
                        'publishLaserScan.inputs:linearDepthData',
                    ),
                    (
                        'readLidarBeams.outputs:intensitiesData',
                        'publishLaserScan.inputs:intensitiesData',
                    ),
                    ('readLidarBeams.outputs:numRows',
                     'publishLaserScan.inputs:numRows'),
                    ('readLidarBeams.outputs:numCols',
                     'publishLaserScan.inputs:numCols'),
                    ('readLidarBeams.outputs:azimuthRange',
                     'publishLaserScan.inputs:azimuthRange'),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ('publishLaserScan.inputs:frameId', 'base_range_sensor_link'),
                    (
                        'publishLaserScan.inputs:topicName',
                        '/scan' if is_ros2 else self.prefix + '/base_scan',
                    ),
                ],
            },
        )
        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath('/lidar_sensor/readLidarBeams'),
            attribute='inputs:lidarPrim',
            target_prim_paths=[self.stage_path +
                               self.prefix + '/base_range_sensor_link/Lidar'],
        )

    def _create_controller_parameter_services(self) -> None:
        self._arm_controller_joints = [
            'arm_lift_joint',
            'arm_flex_joint',
            'arm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint',
        ]
        self._head_controller_joints = [
            'head_pan_joint',
            'head_tilt_joint',
        ]
        self._base_controller_coordinates = [
            'odom_x',
            'odom_y',
            'odom_t',
        ]
        self._gripper_controller_joints = [
            'hand_motor_joint',
        ]

        self._arm_get_parameters_srv = self.ros2node.create_service(
            GetParameters,
            '/arm_trajectory_controller/get_parameters',
            lambda request, context: self._build_parameters_response(
                request, 'joints', self._arm_controller_joints
            ),
        )
        self._head_get_parameters_srv = self.ros2node.create_service(
            GetParameters,
            '/head_trajectory_controller/get_parameters',
            lambda request, context: self._build_parameters_response(
                request, 'joints', self._head_controller_joints
            ),
        )
        self._base_get_parameters_srv = self.ros2node.create_service(
            GetParameters,
            '/omni_base_controller/get_parameters',
            lambda request, context: self._build_parameters_response(
                request, 'base_coordinates', self._base_controller_coordinates
            ),
        )
        self._gripper_get_parameters_srv = self.ros2node.create_service(
            GetParameters,
            '/gripper_controller/get_parameters',
            lambda request, context: self._build_parameters_response(
                request, 'joints', self._gripper_controller_joints
            ),
        )

    def _build_parameters_response(self, request, valid_name, values):
        response = GetParameters.Response()
        for name in request.names:
            pv = ParameterValue()
            if name == valid_name:
                pv.type = ParameterType.PARAMETER_STRING_ARRAY
                pv.string_array_value = list(values)
            else:
                pv.type = ParameterType.PARAMETER_NOT_SET
            response.values.append(pv)
        return response

    def set_base_joint_and_material(self) -> None:
        caster_material = PhysicsMaterial(
            prim_path='/Caster',
            static_friction=0.0,
            dynamic_friction=0.0,
        )
        for l in [
            '/base_l_passive_wheel_z_link/collisions',
            '/base_r_passive_wheel_z_link/collisions',
        ]:
            omni.kit.commands.execute(
                'BindMaterialExt',
                material_path='/Caster',
                prim_path=[self.stage_path + self.prefix + l],
                strength=['weakerThanDescendants'],
                material_purpose='physics',
            )

        tire_material = PhysicsMaterial(
            prim_path='/Tire',
            static_friction=100.0,
            dynamic_friction=100.0,
        )
        for l in ['/base_l_drive_wheel_link/collisions', '/base_r_drive_wheel_link/collisions']:
            omni.kit.commands.execute(
                'BindMaterialExt',
                material_path='/Tire',
                prim_path=[self.stage_path + self.prefix + l],
                strength=['weakerThanDescendants'],
                material_purpose='physics',
            )

        # グリッパ指に高摩擦マテリアルを付ける。既定摩擦だと軽い缶が握っても滑って
        # 抜ける(実測: 0.4cm持ち上げて落ちる)。指の衝突プリム(hand_*/.../collisions)を
        # 自動探索してバインドする(リンク名の取り違え回避)。
        finger_material = PhysicsMaterial(
            prim_path='/GripperFinger',
            static_friction=30.0,
            dynamic_friction=30.0,
        )
        # 摩擦の合成モードを max にする。既定(average)だと缶側の低い摩擦と平均されて
        # 弱まるが、max なら指の高摩擦(8.0)が支配して滑りにくくなる。
        try:
            _fm_prim = stage.get_current_stage().GetPrimAtPath('/GripperFinger')
            _fm_api = PhysxSchema.PhysxMaterialAPI.Apply(_fm_prim)
            _fm_api.CreateFrictionCombineModeAttr().Set('max')
            print('[grip-fric] frictionCombineMode=max', flush=True)
        except Exception as _e:
            print('[grip-fric] combine-mode err %r' % _e, flush=True)
        _root = self.stage_path + self.prefix
        _st = stage.get_current_stage()
        _bound = []
        for _p in _st.Traverse():
            _ps = str(_p.GetPath())
            if _ps.startswith(_root) and 'hand_' in _ps and _ps.endswith('/collisions'):
                try:
                    omni.kit.commands.execute(
                        'BindMaterialExt',
                        material_path='/GripperFinger',
                        prim_path=[_ps],
                        strength=['weakerThanDescendants'],
                        material_purpose='physics',
                    )
                    _bound.append(_ps)
                except Exception as _e:
                    print('[grip-fric] bind fail %s: %r' % (_ps, _e))
        print('[grip-fric] bound finger material to %d prims: %s' % (len(_bound), _bound), flush=True)

        # 指コライダーが動的物体(缶)と接触判定を起こさない問題への対策(実測+GUIで確認:
        # 手のひらは衝突するが指リンクだけ缶を貫通する)。指の collider は convexHull だが、
        # convexHull は cook(凸包の生成計算)に失敗すると enabled のままでも実体の無い
        # 当たり判定になり、見た目だけ残って貫通する(手のひらは cook 成功・指は薄mesh で
        # 失敗、の非対称で説明可)。cook 不要の boundingCube(箱)に変えて確実に実体を作る。
        try:
            from omni.physx.scripts import utils as _pxutils
            _finger_col_prims = [
                self.stage_path + self.prefix + '/hand_l_spring_proximal_link/collisions',
                self.stage_path + self.prefix + '/hand_l_distal_link/collisions',
                self.stage_path + self.prefix + '/hand_r_spring_proximal_link/collisions',
                self.stage_path + self.prefix + '/hand_r_distal_link/collisions',
            ]
            for _cp in _finger_col_prims:
                _cpr = _st.GetPrimAtPath(_cp)
                if not _cpr or not _cpr.IsValid():
                    print('[grip-col] missing %s' % _cp, flush=True)
                    continue
                _pxutils.setCollider(_cpr, approximationShape='convexHull')
                print('[grip-col] convexHull collider (薄いまま) %s' % _cp, flush=True)
        except Exception as _e:
            print('[grip-col] error: %r' % _e, flush=True)

        # グリッパ指関節の駆動を「弱く・ゆっくり」にする。
        # URDF Importer 既定の指ドライブは stiffness(ばね定数)が高く、閉じ指令
        # (position=0)へ一気に駆動するため、軽い自由物体(缶)を弾き飛ばす(実測)。
        # stiffness を下げて握る力を弱め、damping(粘性=動きへの抵抗)を上げて
        # ゆっくり閉じさせ、maxForce(=トルク上限)を絞ることで、「物体に触れたら
        # 弱い力で握って止まる」コンプライアント(柔らかい)な閉じにする。
        # ※ 値は実機合わせ。弾く→さらに下げる / 持ち上げで滑る→少し上げる で調整。
        # 衝突が効くようになったので、握り力を上げて缶を押し付ける(摩擦で保持するため)。
        # damping を上げて閉じをゆっくりにし、缶を揺らさず噛む。
        FINGER_STIFFNESS = 8.0
        FINGER_DAMPING = 28.0
        FINGER_MAX_FORCE = 8.0
        finger_joint_paths = [
            self.stage_path + self.prefix + '/hand_palm_link/hand_l_proximal_joint',
            self.stage_path + self.prefix + '/hand_l_mimic_distal_link/hand_l_distal_joint',
            self.stage_path + self.prefix + '/hand_palm_link/hand_r_proximal_joint',
            self.stage_path + self.prefix + '/hand_r_mimic_distal_link/hand_r_distal_joint',
        ]
        for _fp in finger_joint_paths:
            _fd = UsdPhysics.DriveAPI.Get(
                stage.get_current_stage().GetPrimAtPath(_fp), 'angular'
            )
            if not _fd:
                print('[grip-drive] DriveAPI not found: %s' % _fp, flush=True)
                continue
            _fd.GetStiffnessAttr().Set(FINGER_STIFFNESS)
            _fd.GetDampingAttr().Set(FINGER_DAMPING)
            _fd.GetMaxForceAttr().Set(FINGER_MAX_FORCE)
            print('[grip-drive] tuned %s (k=%.1f d=%.1f maxF=%.1f)'
                  % (_fp, FINGER_STIFFNESS, FINGER_DAMPING, FINGER_MAX_FORCE), flush=True)

        left_passive1_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path
                + self.prefix
                + '/base_l_passive_wheel_x_frame/base_l_passive_wheel_y_frame_joint'
            ),
            'angular',
        )
        left_passive1_drive.GetDampingAttr().Set(0)
        left_passive1_drive.GetStiffnessAttr().Set(0)

        left_passive2_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path
                + self.prefix
                + '/base_l_passive_wheel_y_frame/base_l_passive_wheel_z_joint'
            ),
            'angular',
        )
        left_passive2_drive.GetDampingAttr().Set(0)
        left_passive2_drive.GetStiffnessAttr().Set(0)

        right_passive1_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path
                + self.prefix
                + '/base_r_passive_wheel_x_frame/base_r_passive_wheel_y_frame_joint'
            ),
            'angular',
        )
        right_passive1_drive.GetDampingAttr().Set(0)
        right_passive1_drive.GetStiffnessAttr().Set(0)

        right_passive2_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path
                + self.prefix
                + '/base_r_passive_wheel_y_frame/base_r_passive_wheel_z_joint'
            ),
            'angular',
        )
        right_passive2_drive.GetDampingAttr().Set(0)
        right_passive2_drive.GetStiffnessAttr().Set(0)

        left_wheel_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path + self.prefix + '/base_roll_link/base_l_drive_wheel_joint'
            ),
            'angular',
        )
        right_wheel_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path + self.prefix + '/base_roll_link/base_r_drive_wheel_joint'
            ),
            'angular',
        )
        roll_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path + self.prefix + '/base_link/base_roll_joint'
            ),
            'angular',
        )

        left_wheel_drive.GetDampingAttr().Set(15000)
        right_wheel_drive.GetDampingAttr().Set(15000)
        roll_drive.GetDampingAttr().Set(15000)

        left_wheel_drive.GetStiffnessAttr().Set(0)
        right_wheel_drive.GetStiffnessAttr().Set(0)
        roll_drive.GetStiffnessAttr().Set(0)

    def on_cmd_vel(self, msg):
        if self.simulation_context is not None:
            self.cmd_vel_msg = msg
            self.last_cmd_vel_time = self.simulation_context.current_time

    def on_laserscan_odom(self, msg):
        posestamped = PoseStamped()
        posestamped.header.stamp = msg.header.stamp
        posestamped.header.frame_id = 'world'
        posestamped.pose = msg.pose.pose
        self.on_laserscan_pose(posestamped)

    def on_laserscan_pose(self, msg):
        if not is_ros2:
            odom = Odometry()
            odom.header.stamp = msg.header.stamp
            odom.header.frame_id = 'world'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose = msg.pose
            odom.pose.covariance = [
                0.001,
                0,
                0,
                0,
                0,
                0,
                0,
                0.001,
                0,
                0,
                0,
                0,
                0,
                0,
                100000.0,
                0,
                0,
                0,
                0,
                0,
                0,
                100000.0,
                0,
                0,
                0,
                0,
                0,
                0,
                100000.0,
                0,
                0,
                0,
                0,
                0,
                0,
                1000.0,
            ]
            self.laser_odom_pub.publish(odom)

        q = msg.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)[2]
        self.odometry_estimator.set_pose(
            msg.pose.position.x, msg.pose.position.y, yaw)

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_ros_time(
            self.simulation_context.current_time)
        js.name = list(self._joints.keys())
        for n in js.name:
            (joint, joint_type, inv) = self._joints[n]
            st = self.dc.get_dof_state(joint, _dynamic_control.STATE_ALL)
            if joint_type == _dynamic_control.JOINT_PRISMATIC:
                st.pos = st.pos * get_stage_units()
            if inv:
                st.pos = -st.pos
                st.vel = -st.vel
                st.effort = -st.effort
            js.position.append(st.pos)
            js.velocity.append(st.vel)
            js.effort.append(st.effort * 1000.0)
        if not is_ros2:
            js.name = js.name + ['odom_x', 'odom_y', 'odom_t']
            js.position.extend([
                self.odometry_estimator.pose.x,
                self.odometry_estimator.pose.y,
                self.odometry_estimator.pose.ang,
            ])
            js.velocity.extend([0, 0, 0])
            js.effort.extend([0, 0, 0])
        self.joint_state_pub.publish(js)

    def _object_body_paths(self):
        # ロボット以外の剛体(=掴める物体)プリムのパス一覧を一度だけ集めてキャッシュ。
        if self._obj_paths_cache is not None:
            return self._obj_paths_cache
        paths = []
        try:
            _st = stage.get_current_stage()
            _root = self.stage_path + self.prefix
            for _p in _st.Traverse():
                _ps = str(_p.GetPath())
                if _ps.startswith(_root):
                    continue  # ロボット自身は除く
                if _p.HasAPI(UsdPhysics.RigidBodyAPI):
                    paths.append(_ps)
        except Exception as _e:
            print('[graspA] object scan err %r' % _e, flush=True)
        self._obj_paths_cache = paths
        print('[graspA] graspable bodies: %s' % paths, flush=True)
        return paths

    def _set_grasp_object_collision(self, body_path, enabled):
        """把持中の物体の衝突を一時的に切る/戻す。

        案A(アタッチ把持)は物体を毎ステップ手の位置へテレポートして保持する。物体の
        衝突(recol で有効化)が残っていると、テレポート先で指と毎ステップ押し合い、
        移動時に発散してロボットごと吹き飛ぶ。把持中は衝突を切り、離したら戻す。
        """
        try:
            import omni.usd
            from pxr import Usd, UsdPhysics
            _st = omni.usd.get_context().get_stage()
            _root = _st.GetPrimAtPath(body_path)
            if not _root or not _root.IsValid():
                return
            for _p in Usd.PrimRange(_root):
                if _p.HasAPI(UsdPhysics.CollisionAPI):
                    _a = _p.GetAttribute('physics:collisionEnabled')
                    if not _a:
                        _a = UsdPhysics.CollisionAPI(_p).CreateCollisionEnabledAttr()
                    _a.Set(bool(enabled))
        except Exception as _e:
            print('[graspA] collision toggle err %r' % _e, flush=True)

    def _grasp_attach_update(self):
        # 案A: 指の衝突が効かない問題を迂回して把持を再現する。
        # グリッパが閉じていて把持中心の近くに物体があれば、その物体をグリッパに
        # 追従させる(=掴む)。グリッパが開いたら追従を止める(=離す)。
        if not getattr(self, '_joints', None) or 'hand_motor_joint' not in self._joints:
            return
        try:
            _hm = self.dc.get_dof_state(
                self._joints['hand_motor_joint'][0], _dynamic_control.STATE_POS).pos
        except Exception:
            return
        if self._palm_body is None:
            self._palm_body = self.dc.get_rigid_body(
                self.stage_path + self.prefix + '/hand_palm_link')
        if getattr(self, '_lfinger_body', None) is None:
            self._lfinger_body = self.dc.get_rigid_body(
                self.stage_path + self.prefix + '/hand_l_distal_link')
            self._rfinger_body = self.dc.get_rigid_body(
                self.stage_path + self.prefix + '/hand_r_distal_link')
        if not self._palm_body or not self._lfinger_body or not self._rfinger_body:
            return
        _palm = self.dc.get_rigid_body_pose(self._palm_body)
        _pp = (_palm.p.x, _palm.p.y, _palm.p.z)
        _pq = (_palm.r.x, _palm.r.y, _palm.r.z, _palm.r.w)
        # 把持中心 = 左右の指先(distal)の中点。物体が来るべき場所そのもの。
        _lf = self.dc.get_rigid_body_pose(self._lfinger_body).p
        _rf = self.dc.get_rigid_body_pose(self._rfinger_body).p
        _gc = ((_lf.x + _rf.x) / 2.0, (_lf.y + _rf.y) / 2.0, (_lf.z + _rf.z) / 2.0)

        CLOSE_T, OPEN_T, GRASP_DIST = 0.5, 0.6, 0.15
        if self._grasp_obj is None:
            if _hm < CLOSE_T:  # 閉じている/閉じ動作中
                _best = None
                _bestd = GRASP_DIST
                _mind = 999.0
                for _bp in self._object_body_paths():
                    _h = self.dc.get_rigid_body(_bp)
                    if not _h:
                        continue
                    _op = self.dc.get_rigid_body_pose(_h)
                    _d = math.sqrt((_gc[0] - _op.p.x) ** 2 + (_gc[1] - _op.p.y) ** 2
                                   + (_gc[2] - _op.p.z) ** 2)
                    _mind = min(_mind, _d)
                    if _d < _bestd:
                        _bestd = _d
                        _best = (_bp, _h, _op)
                if _mind < 0.4 and not getattr(self, '_grasp_dbg_done', False):
                    print('[graspA] try: gc=(%.3f,%.3f,%.3f) nearest_obj_dist=%.3f (閾値%.2f)'
                          % (_gc[0], _gc[1], _gc[2], _mind, GRASP_DIST), flush=True)
                    self._grasp_dbg_done = True
                if _best is not None:
                    _bp, _h, _op = _best
                    _rel_p = _q_rot(_q_conj(_pq),
                                    (_op.p.x - _pp[0], _op.p.y - _pp[1], _op.p.z - _pp[2]))
                    _rel_q = _q_mul(_q_conj(_pq), (_op.r.x, _op.r.y, _op.r.z, _op.r.w))
                    self._grasp_obj = {'h': _h, 'rel_p': _rel_p, 'rel_q': _rel_q, 'path': _bp}
                    # 把持中はテレポート保持なので衝突を切る(指との押し合いで発散しない)。
                    self._set_grasp_object_collision(_bp, False)
                    print('[graspA] 掴んだ: %s (dist=%.3f)' % (_bp, _bestd), flush=True)
        else:
            if _hm > OPEN_T:  # 開いた → 離す
                try:
                    self.dc.set_rigid_body_linear_velocity(self._grasp_obj['h'], (0.0, 0.0, 0.0))
                    self.dc.set_rigid_body_angular_velocity(self._grasp_obj['h'], (0.0, 0.0, 0.0))
                except Exception:
                    pass
                # 離したら衝突を戻す(机に乗る/他物体と当たる)。
                self._set_grasp_object_collision(self._grasp_obj['path'], True)
                print('[graspA] 離した: %s' % self._grasp_obj['path'], flush=True)
                self._grasp_obj = None
                self._grasp_dbg_done = False  # 次の閉じで再びデバッグ出力
            else:  # 掴んだまま → palm に追従
                _g = self._grasp_obj
                _wp = _q_rot(_pq, _g['rel_p'])
                _wp = (_pp[0] + _wp[0], _pp[1] + _wp[1], _pp[2] + _wp[2])
                _wq = _q_mul(_pq, _g['rel_q'])
                _t = _dynamic_control.Transform()
                _t.p = _wp
                _t.r = _wq
                self.dc.set_rigid_body_pose(_g['h'], _t)
                self.dc.set_rigid_body_linear_velocity(_g['h'], (0.0, 0.0, 0.0))
                self.dc.set_rigid_body_angular_velocity(_g['h'], (0.0, 0.0, 0.0))

    def onsimulationstart(self, simulation_context):
        self.simulation_context = simulation_context
        self.prev_time = self.simulation_context.current_time
        # 案A(アタッチ把持)用の状態
        self._grasp_obj = None        # 掴んでいる物体 {h, rel_p, rel_q, path}
        self._palm_body = None        # hand_palm_link の剛体ハンドル(キャッシュ)
        self._obj_paths_cache = None  # 物体(剛体)プリムのパス一覧(キャッシュ)

    def step(self):
        og.Controller.set(
            og.Controller.attribute(
                '/ros_controllers/OnImpulseEvent.state:enableImpulse'), True
        )

        dt = self.simulation_context.current_time - self.prev_time

        if not self.art:
            self.art = self.dc.get_articulation(self.stage_path + self.prefix)
            if self.art == _dynamic_control.INVALID_HANDLE:
                print('{self.prefix} is not an articulation')
            self._joints = {}
            for i in range(self.dc.get_articulation_dof_count(self.art)):
                dof_ptr = self.dc.get_articulation_dof(self.art, i)
                if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                    dof_name = self.dc.get_dof_name(dof_ptr)
                    joint = self.dc.find_articulation_joint(self.art, dof_name)
                    self._joints[dof_name] = (
                        dof_ptr,
                        self.dc.get_joint_type(joint),
                        dof_name
                        in [
                            'arm_flex_joint',
                            'arm_lift_joint',
                            'wrist_flex_joint',
                            'arm_roll_joint',
                        ],
                    )
            print(self._joints)
            self.left_wheel_ptr = self.dc.find_articulation_dof(
                self.art, 'base_l_drive_wheel_joint'
            )
            self.right_wheel_ptr = self.dc.find_articulation_dof(
                self.art, 'base_r_drive_wheel_joint'
            )
            self.roll_ptr = self.dc.find_articulation_dof(
                self.art, 'base_roll_joint')
            self.robots.initialize()

        self.dc.wake_up_articulation(self.art)

        self.publish_joint_states()

        # 案A: グリッパが閉じて物体が近ければ掴んで追従、開いたら離す
        # (正攻法=物理衝突の検証中は _ATTACH_GRASP_ENABLED=False で無効化)
        if _ATTACH_GRASP_ENABLED:
            self._grasp_attach_update()

        left_state = self.dc.get_dof_state(
            self.left_wheel_ptr, _dynamic_control.STATE_ALL)
        right_state = self.dc.get_dof_state(
            self.right_wheel_ptr, _dynamic_control.STATE_ALL)
        roll_state = self.dc.get_dof_state(
            self.roll_ptr, _dynamic_control.STATE_ALL)

        state_ = VehicleState()
        state_.steer_angle = roll_state.pos
        joint_param_ = JointSpace()
        joint_param_.vel_wheel_l = left_state.vel
        joint_param_.vel_wheel_r = right_state.vel
        joint_param_.vel_steer = roll_state.vel

        cartesian_param_ = self.vehicle_dynamics.forward(joint_param_, state_)
        abs_dot_x, abs_dot_y = self.odometry_estimator.integrate(
            cartesian_param_, dt)

        odom = Odometry()
        odom.header.stamp = self.get_ros_time(
            self.simulation_context.current_time)
        odom.header.frame_id = 'odom' if is_ros2 else 'world'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.odometry_estimator.pose.x
        odom.pose.pose.position.y = self.odometry_estimator.pose.y
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.odometry_estimator.pose.ang)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom.pose.covariance = [
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            100000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            100000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            100000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1000.0,
        ]
        odom.twist.twist.linear.x = abs_dot_x
        odom.twist.twist.linear.y = abs_dot_y
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = cartesian_param_.dot_r
        odom.twist.covariance = [
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            100000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            100000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            100000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1000.0,
        ]
        self.base_odom_pub.publish(odom)

        cmd = CartSpace()
        if self.odom_trajectory_action_server._action_goal is not None:
            self._base_hold_pose = None  # 追従中はホールド解除
            self.odom_trajectory_action_server._odometry = self.odometry_estimator.pose
            cmd.dot_x = (
                self.odom_trajectory_action_server._joints['odom_x']
                - self.odometry_estimator.pose.x
            )
            cmd.dot_y = (
                self.odom_trajectory_action_server._joints['odom_y']
                - self.odometry_estimator.pose.y
            )
            cmd.dot_r = (
                self.odom_trajectory_action_server._joints['odom_t']
                - self.odometry_estimator.pose.ang
            )
        elif (
            self.last_cmd_vel_time + 2.0 > self.simulation_context.current_time
            and self.cmd_vel_msg is not None
        ):
            self._base_hold_pose = None  # 手動指令中はホールド解除
            ang = self.odometry_estimator.pose.ang + 0.5 * self.cmd_vel_msg.angular.z * dt
            cosr = math.cos(ang)
            sinr = math.sin(ang)
            cmd.dot_x = self.cmd_vel_msg.linear.x * cosr - self.cmd_vel_msg.linear.y * sinr
            cmd.dot_y = self.cmd_vel_msg.linear.x * sinr + self.cmd_vel_msg.linear.y * cosr
            cmd.dot_r = self.cmd_vel_msg.angular.z
        else:
            # 無指令時は速度0で停止する(pull前の挙動)。
            #
            # 下の「その場保持 P 制御」は本来、把持時に台車が振動するのを抑えるために
            # 追加された。しかし副作用として、スポーン直後などにオドメトリの微小誤差を
            # P制御が追いかけ、一度動き出すと止まらず台車がひとりでに動く問題があった。
            # そのため既定では無効化し、速度0指令(=停止)に戻す。
            # 把持時の台車振動を抑えたいときは、下のブロックのコメントを外して復活させる。
            cmd.dot_x = 0.0
            cmd.dot_y = 0.0
            cmd.dot_r = 0.0
            # --- 把持時の振動抑制用「その場保持P制御」(必要なときだけ有効化) ---
            # if getattr(self, '_base_hold_pose', None) is None:
            #     self._base_hold_pose = (
            #         self.odometry_estimator.pose.x,
            #         self.odometry_estimator.pose.y,
            #         self.odometry_estimator.pose.ang,
            #     )
            # hx, hy, ht = self._base_hold_pose
            # cmd.dot_x = hx - self.odometry_estimator.pose.x
            # cmd.dot_y = hy - self.odometry_estimator.pose.y
            # cmd.dot_r = math.atan2(
            #     math.sin(ht - self.odometry_estimator.pose.ang),
            #     math.cos(ht - self.odometry_estimator.pose.ang))

        relcmd = CartSpace()
        diff_r = cmd.dot_r * dt
        ang = self.odometry_estimator.pose.ang + 0.5 * diff_r
        cosr = math.cos(-ang)
        sinr = math.sin(-ang)
        relcmd.dot_x = cmd.dot_x * cosr - cmd.dot_y * sinr
        relcmd.dot_y = cmd.dot_x * sinr + cmd.dot_y * cosr
        # diff_r = cmd.dot_r * dt なので diff_r/dt は数学的に cmd.dot_r と同じ。
        # ただし dt=0(タイムライン一時停止中など)だと 0/0 でゼロ割りクラッシュになり、
        # step() が毎フレーム落ちてログが溢れる。割り算をやめて等価な cmd.dot_r を使う。
        relcmd.dot_r = cmd.dot_r

        jcmd = self.vehicle_dynamics.inverse(relcmd, state_)

        ratio = abs(jcmd.vel_steer) / self.vel_limit_steer_
        ratio = max(ratio, abs(jcmd.vel_wheel_l) / self.vel_limit_wheel_)
        ratio = max(ratio, abs(jcmd.vel_wheel_r) / self.vel_limit_wheel_)
        if ratio > 1.0:
            jcmd.vel_steer /= ratio
            jcmd.vel_wheel_l /= ratio
            jcmd.vel_wheel_r /= ratio

        self.dc.set_dof_velocity_target(self.left_wheel_ptr, jcmd.vel_wheel_l)
        self.dc.set_dof_velocity_target(self.right_wheel_ptr, jcmd.vel_wheel_r)
        self.dc.set_dof_velocity_target(self.roll_ptr, jcmd.vel_steer)

        force_readings = self.robots.get_measured_joint_forces(
            joint_indices=[
                self.robots._metadata.joint_indices['wrist_ft_sensor_frame_joint'] + 1]
        )
        wrench = WrenchStamped()
        wrench.header.stamp = self.get_ros_time(
            self.simulation_context.current_time)
        wrench.header.frame_id = 'wrist_ft_sensor_frame'
        wrench.wrench.force.x = float(force_readings[0][0][0])
        wrench.wrench.force.y = float(force_readings[0][0][1])
        wrench.wrench.force.z = float(force_readings[0][0][2])
        wrench.wrench.torque.x = float(force_readings[0][0][3])
        wrench.wrench.torque.y = float(force_readings[0][0][4])
        wrench.wrench.torque.z = float(force_readings[0][0][5])
        self.ft_sensor_pub.publish(wrench)

        # gravity-compensated wrench: EMA ベースライン(重力)を差し引く。
        raw6 = [float(force_readings[0][0][i]) for i in range(6)]
        if not self._wrench_bias_inited:
            self._wrench_bias = list(raw6)
            self._wrench_bias_inited = True
        else:
            # alpha 小さめ: ゆっくりの重力変化は追従、接触の急変は残す。
            a = 0.02
            for i in range(6):
                self._wrench_bias[i] += a * (raw6[i] - self._wrench_bias[i])
        comp = WrenchStamped()
        comp.header.stamp = wrench.header.stamp
        comp.header.frame_id = 'wrist_ft_sensor_frame'
        comp.wrench.force.x = raw6[0] - self._wrench_bias[0]
        comp.wrench.force.y = raw6[1] - self._wrench_bias[1]
        comp.wrench.force.z = raw6[2] - self._wrench_bias[2]
        comp.wrench.torque.x = raw6[3] - self._wrench_bias[3]
        comp.wrench.torque.y = raw6[4] - self._wrench_bias[4]
        comp.wrench.torque.z = raw6[5] - self._wrench_bias[5]
        self.ft_sensor_comp_pub.publish(comp)

        self.arm_trajectory_action_server.step(dt=dt)
        self.head_trajectory_action_server.step(dt=dt)
        self.odom_trajectory_action_server.step(dt=dt)
        self.gripper_trajectory_action_server.step(dt=dt)
        self.gripper_apply_force_action_server.step(dt=dt)
        self.gripper_command_action_server.step(dt=dt)

        self.prev_time = self.simulation_context.current_time
