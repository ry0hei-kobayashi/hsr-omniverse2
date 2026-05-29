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
        self._inverse_direction = False

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
            target_effort = self._action_goal.effort
            if self._inverse_direction:
                target_effort = -target_effort

            self.dci.wake_up_articulation(self._articulation)
            for name in self._joints:
                if target_effort >= 0.0:
                    self._set_joint_position(name, 0.0)
                else:
                    self._set_joint_position(name, math.pi)

            effort = 0
            effort_reached = True
            for name in self._joints:
                effort = self._get_joint_effort(name)
                if abs(effort) - abs(target_effort) < 0.0:
                    effort_reached = False
                    break
            if effort_reached:
                self._action_goal = None
                self._action_result_message.effort = effort
                self._action_result_message.stalled = False
                if self._action_goal_handle is not None:
                    if is_ros2 is False:
                        self._action_goal_handle.set_succeeded(
                            self._action_result_message)
                    else:
                        self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                return

            current_position_sum = 0
            for name in self._joints:
                position = self._get_joint_position(name)
                current_position_sum += position
            if abs(current_position_sum - self._action_previous_position_sum) < 1e-6:
                self._action_goal = None
                self._action_result_message.effort = effort
                self._action_result_message.stalled = True
                if self._action_goal_handle is not None:
                    if is_ros2 is False:
                        self._action_goal_handle.set_aborted(
                            self._action_result_message)
                    else:
                        self._action_goal_handle.abort()
                    self._action_goal_handle = None
                return
            self._action_previous_position_sum = current_position_sum

            time_passed = self._get_time() - self._action_start_time
            if time_passed >= self._action_timeout:
                self._action_goal = None
                if self._action_goal_handle is not None:
                    if is_ros2 is False:
                        self._action_goal_handle.set_aborted()
                    else:
                        self._action_goal_handle.abort()
                    self._action_goal_handle = None


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
        self.hsr = stage.add_reference_to_stage(
            os.path.dirname(os.path.abspath(__file__)) +
            '/usd/hsrb/hsrb4s.usd',
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
        self.ft_sensor_pub = self.create_publisher(
            self.prefix + '/wrist_wrench/raw', WrenchStamped)

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
            horizontal_resolution=1.0,
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

    def onsimulationstart(self, simulation_context):
        self.simulation_context = simulation_context
        self.prev_time = self.simulation_context.current_time

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
            ang = self.odometry_estimator.pose.ang + 0.5 * self.cmd_vel_msg.angular.z * dt
            cosr = math.cos(ang)
            sinr = math.sin(ang)
            cmd.dot_x = self.cmd_vel_msg.linear.x * cosr - self.cmd_vel_msg.linear.y * sinr
            cmd.dot_y = self.cmd_vel_msg.linear.x * sinr + self.cmd_vel_msg.linear.y * cosr
            cmd.dot_r = self.cmd_vel_msg.angular.z

        relcmd = CartSpace()
        diff_r = cmd.dot_r * dt
        ang = self.odometry_estimator.pose.ang + 0.5 * diff_r
        cosr = math.cos(-ang)
        sinr = math.sin(-ang)
        relcmd.dot_x = cmd.dot_x * cosr - cmd.dot_y * sinr
        relcmd.dot_y = cmd.dot_x * sinr + cmd.dot_y * cosr
        relcmd.dot_r = diff_r / dt

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

        self.arm_trajectory_action_server.step(dt=dt)
        self.head_trajectory_action_server.step(dt=dt)
        self.odom_trajectory_action_server.step(dt=dt)
        self.gripper_trajectory_action_server.step(dt=dt)
        self.gripper_apply_force_action_server.step(dt=dt)
        self.gripper_command_action_server.step(dt=dt)

        self.prev_time = self.simulation_context.current_time
