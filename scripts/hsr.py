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

from hsr_action_servers import (arm_trajectory_action_server, gripper_apply_force_action_server, gripper_command_action_server, gripper_trajectory_action_server, head_trajectory_action_server, odom_trajectory_action_server)
from hsr_base_drive import BaseDriveMixin
from hsr_grasp import GraspMixin
from hsr_math import euler_from_quaternion, quaternion_from_euler  # noqa: F401
from hsr_physics import PhysicsTuningMixin
from hsr_sensors import SensorsMixin, og_ros_camera_info_node  # noqa: F401 (hsr_hsrc_ex が base.og_ros_camera_info_node を参照)
from hsr_params import (_ATTACH_GRASP_ENABLED, _BASE_ANGULAR_ACCEL_LIMIT, _BASE_BRAKE_ANGULAR_C, _BASE_BRAKE_ANGULAR_K, _BASE_BRAKE_C, _BASE_BRAKE_ENABLED, _BASE_BRAKE_ENGAGE_ANGULAR, _BASE_BRAKE_ENGAGE_LINEAR, _BASE_BRAKE_K, _BASE_CMD_TAU, _BASE_CMD_VEL_TIMEOUT, _BASE_CONTROL_DEADBAND, _BASE_DIAG, _BASE_DIRECT_DRIVE, _BASE_GOAL_TOLERANCE, _BASE_KINEMATIC_DRIVE, _BASE_LINEAR_ACCEL_LIMIT, _BASE_PHYSICS_REPORT, _BASE_ROT_NO_VELOCITY_WRITE, _BASE_STEER_ACCEL_LIMIT, _BASE_TRAJ_D_GAIN, _BASE_TRAJ_P_GAIN, _BASE_VELOCITY_WRITE, _BASE_WHEEL_ACCEL_LIMIT, _GT_ODOM, _OBJECT_POSE_REPORT, _OBJECT_POSE_REPORT_AT)
from hsr_kinematics import CartSpace, JointSpace, VehicleDynamics, VehicleState, WheelOdometry, hsr_config


extensions.enable_extension('isaacsim.ros2.bridge')

extensions.enable_extension('isaacsim.sensors.physx')
extensions.enable_extension('isaacsim.util.debug_draw')


if not _ATTACH_GRASP_ENABLED:
    print('[graspA] アタッチ把持は無効 (既定)。物理接触のみで把持します。'
          ' 有効にするには make up GRASP=1', flush=True)


# 台車の駆動系寸法 [m] の既定値(= HSR-B base_v2 の URDF 実寸)。VehicleDynamics
# (速度↔車輪/操舵)と車輪オドメトリが使う。robot ごとに実寸が違うので、実際には
# hsr クラスのクラス属性 WHEEL_* を使い、hsrc_ex は hsr_hsrc_ex 側で上書きする
# (共有コードに片方の robot の値を直書きしない)。
wheel_separation = 0.266   # HSR-B = 2 x drive_wheel_offset_y(0.133)
wheel_radius = 0.04        # HSR-B drive_wheel_radius
wheel_offset = 0.11        # HSR-B drive_wheel_offset_x


class hsr(SensorsMixin, PhysicsTuningMixin, GraspMixin, BaseDriveMixin):
    # 台車の駆動系寸法 [m]。既定は HSR-B(base_v2)。hsrc_ex は hsr_hsrc_ex.hsr が
    # これらを上書きする(URDF base_v0 の実寸)。VehicleDynamics と車輪オドメトリが使う。
    WHEEL_RADIUS = wheel_radius
    WHEEL_SEPARATION = wheel_separation
    WHEEL_OFFSET = wheel_offset

    def __init__(self, prefix='/hsrb', stage_path='/World', config=None) -> None:
        if config is None:
            config = hsr_config()


        if not rclpy.ok():
            rclpy.init()
        self.ros2node = rclpy.node.Node('isaac_sim_hsr')
        # Trajectory time_from_start is expressed in simulation time.  The
        # simulator can run below real time, so a wall-clock controller
        # advances targets too quickly relative to PhysX and excites the
        # mobile base.
        self.ros2node.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True),
        ])
        self.create_subscriber = lambda t, d, c: self.ros2node.create_subscription(
            d, t, c, qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.create_publisher = lambda t, d: self.ros2node.create_publisher(
            d, t, qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.create_publisher_reliable = lambda t, d: self.ros2node.create_publisher(
            d, t, qos_profile=rclpy.qos.qos_profile_system_default
        )
        self.clock_pub = self.ros2node.create_publisher(
            Clock, '/clock', 1)
        self.get_ros_time = lambda t: rclpy.time.Time(seconds=t).to_msg()
        self.tf_broadcaster = TransformBroadcaster(self.ros2node)
        self._create_controller_parameter_services()

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.ros2node)
        self._executor = executor
        self._executor_thread = threading.Thread(
            target=executor.spin, daemon=True)
        self._executor_thread.start()

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

        self.laserscan_pose_sub = self.create_subscriber(
            '/laser_odom', Odometry, self.on_laserscan_odom
        )
        self.base_odom_pub = self.create_publisher_reliable(
            '/omni_base_controller/wheel_odom', Odometry
        )

        self.robots = Articulation(
            prim_paths_expr=self.stage_path + self.prefix, name='hsr_view'
        )
        # wrench は RELIABLE で出す。実機の /hsrb/wrist_wrench/* は RELIABLE で、
        # skill の is_hand_collision は rclpy.wait_for_message を既定QoS(RELIABLE)で
        # 購読する。BEST_EFFORT で出すと QoS 不一致でメッセージが届かず
        # wait_for_message が永久ブロックする (実測)。
        self.ft_sensor_pub = self.create_publisher_reliable(
            self.prefix + '/wrist_wrench/raw', WrenchStamped
        )
        # 重力補正済み wrench。実機は compensated を出すので skill は
        # こちらを優先購読する。EMA で重力(=ゆっくり変化)を差し引き、
        # 接触の過渡だけ残す。
        self.ft_sensor_comp_pub = self.create_publisher_reliable(
            self.prefix + '/wrist_wrench/compensated', WrenchStamped
        )
        self._wrench_bias = [0.0] * 6
        self._wrench_bias_inited = False

        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        self.cmd_vel_msg = None
        self.last_cmd_vel_time = 0.0
        self.create_subscriber(
            '/omni_base_controller/cmd_vel',
            Twist,
            self.on_cmd_vel,
        )
        self.vehicle_dynamics = VehicleDynamics(
            self.WHEEL_RADIUS, self.WHEEL_SEPARATION, self.WHEEL_OFFSET)
        self.odometry_estimator = WheelOdometry()
        # Match hsrb_base_controllers.  Scaling all three axes by the most
        # restrictive limit preserves the twin-caster kinematic ratio.
        self.vel_limit_steer_ = 1.8
        self.vel_limit_wheel_ = 8.5
        self._base_cmd_prev = JointSpace()
        self._base_direct_cmd_prev = CartSpace()
        self._base_direct_applied_cmd = CartSpace()
        self._base_direct_pose = None
        self._base_kinematic_velocity = (0.0, 0.0, 0.0)
        self._base_hold_pose = None
        self._base_brake_engaged = False
        self._base_brake_pin = None
        self._base_setpoint = None
        self._base_pose_error_integral = [0.0, 0.0, 0.0]
        self._base_trajectory_was_active = False

        self.joint_state_pub = self.create_publisher_reliable(
            '/joint_states', JointState
        )

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

        q = msg.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)[2]
        # 【無効化】base の odom をレーザ pose で上書きすると、scan が不良(壁を捉え
        # られず最大レンジばかり)なときにこの pose がドリフトし、台車の軌道P制御が
        # それを追いかけて指令ゼロでもゆっくり回り続ける原因になる。
        # Sim では車輪オドメトリ(WheelOdometry.integrate)が正確なので base odom は
        # それに任せ、ここでのレーザ上書きは行わない。地図との整合は別の
        # localization(map->odom)が担当する想定。
        # self.odometry_estimator.set_pose(
        #     msg.pose.position.x, msg.pose.position.y, yaw)

    def publish_joint_states(self, dt):
        js = JointState()
        js.header.stamp = self.get_ros_time(
            self.simulation_context.current_time)
        js.name = list(self._joints.keys())
        previous_positions = getattr(
            self, '_joint_state_previous_positions', {})
        filtered_velocities = getattr(
            self, '_joint_state_filtered_velocities', {})
        for n in js.name:
            (joint, joint_type, inv) = self._joints[n]
            st = self.dc.get_dof_state(joint, _dynamic_control.STATE_ALL)
            position = st.pos
            if joint_type == _dynamic_control.JOINT_PRISMATIC:
                position *= get_stage_units()
            if inv:
                position = -position
                st.effort = -st.effort
            previous_position = previous_positions.get(n)
            previous_velocity = filtered_velocities.get(n, 0.0)
            if previous_position is None or dt <= 1e-9:
                velocity = 0.0
            else:
                position_delta = position - previous_position
                if joint_type == _dynamic_control.JOINT_REVOLUTE:
                    position_delta = math.atan2(
                        math.sin(position_delta),
                        math.cos(position_delta),
                    )
                raw_velocity = position_delta / dt
                alpha = min(1.0, dt / (0.05 + dt))
                velocity = previous_velocity + alpha * (
                    raw_velocity - previous_velocity)
            previous_positions[n] = position
            filtered_velocities[n] = velocity
            js.position.append(position)
            js.velocity.append(velocity)
            js.effort.append(st.effort * 1000.0)
        self._joint_state_previous_positions = previous_positions
        self._joint_state_filtered_velocities = filtered_velocities
        self.joint_state_pub.publish(js)


    def onsimulationstart(self, simulation_context):
        self.simulation_context = simulation_context
        self.prev_time = self.simulation_context.current_time
        self._joint_state_previous_positions = {}
        self._joint_state_filtered_velocities = {}
        # 案A(アタッチ把持)用の状態
        self._grasp_obj = None  # 掴んでいる物体 {h, rel_p, rel_q, path}
        self._palm_body = None  # hand_palm_link の剛体ハンドル(キャッシュ)
        self._obj_paths_cache = None  # 物体(剛体)プリムのパス一覧(キャッシュ)


    def step(self):
        clock = Clock()
        clock.clock = self.get_ros_time(
            self.simulation_context.current_time)
        self.clock_pub.publish(clock)

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
            self._base_root_body = self.dc.get_articulation_root_body(self.art)
            self.robots.initialize()

            for _name in (
                'arm_lift_joint',
                'torso_lift_joint',
                'arm_flex_joint',
                'arm_roll_joint',
                'wrist_flex_joint',
                'wrist_roll_joint',
            ):
                _ptr = self._joints[_name][0]
                _props = self.dc.get_dof_properties(_ptr)
                print(
                    '[arm-runtime] %s k=%.6g d=%.6g maxEffort=%.6g '
                    'maxVelocity=%.6g'
                    % (
                        _name,
                        _props.stiffness,
                        _props.damping,
                        _props.max_effort,
                        _props.max_velocity,
                    ),
                    flush=True,
                )

            # for reset world
            self._home_dof_pos = {
                _n: self.dc.get_dof_state(
                    _p[0], _dynamic_control.STATE_POS).pos
                for _n, _p in self._joints.items()
            }

            if _BASE_PHYSICS_REPORT:
                self._dump_physics_report()

        self.dc.wake_up_articulation(self.art)

        # 物体は spawn 後に落下・転がりで位置が変わる。落ち着いた頃に一度だけ
        # 実際の座標を出しておくと、把持の目標値をそこから決められる。
        if (
            _OBJECT_POSE_REPORT
            and not getattr(self, '_obj_pose_reported', False)
            and self.simulation_context.current_time > _OBJECT_POSE_REPORT_AT
        ):
            self._obj_pose_reported = True
            self._dump_object_poses()

        self.publish_joint_states(dt)

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

        if _BASE_DIRECT_DRIVE:
            # In direct mode the wheel joints are visual/passive; odometry
            # velocity must come from the articulation root, not wheel motion.
            # Read the same Tensor API state used below for commands; the
            # Dynamic Control rigid-body setter is reduced by the fixed-link
            # constraint solve before it becomes articulation root velocity.
            cartesian_param_ = CartSpace()
            if _BASE_KINEMATIC_DRIVE:
                _kv = getattr(
                    self, '_base_kinematic_velocity', (0.0, 0.0, 0.0))
                abs_dot_x, abs_dot_y = float(_kv[0]), float(_kv[1])
                cartesian_param_.dot_r = float(_kv[2])
            else:
                root_velocity = self.robots.get_velocities()[0]
                abs_dot_x = float(root_velocity[0])
                abs_dot_y = float(root_velocity[1])
                cartesian_param_.dot_r = float(root_velocity[5])
            if not _GT_ODOM and dt > 0.0:
                self.odometry_estimator.pose.x += abs_dot_x * dt
                self.odometry_estimator.pose.y += abs_dot_y * dt
                self.odometry_estimator.pose.ang += (
                    cartesian_param_.dot_r * dt)
        else:
            cartesian_param_ = self.vehicle_dynamics.forward(
                joint_param_, state_)
            abs_dot_x, abs_dot_y = self.odometry_estimator.integrate(
                cartesian_param_, dt)

        # === Phase2: odometry_estimator.pose を物理の真値で上書き ===
        # これ以降の wheel_odom発行(2195)・/omni_base_controller/state・odom_x/y/t(→計画)・
        # 軌道追従制御 が全て真値の台車位置を使う(車輪スリップ/レーザ破綻を回避)。
        # twist(abs_dot_x/y)は integrate 値のまま。回転中心 base_footprint を読む。
        # odom≠world のオフセットは初回に一度だけ 真world↔現在odom を記録して吸収。
        if _GT_ODOM:
            try:
                if getattr(self, '_gt_body', None) is None:
                    _bf = self.dc.get_rigid_body(
                        self.stage_path + self.prefix + '/base_footprint')
                    self._gt_body = _bf if _bf else \
                        self.dc.get_articulation_root_body(self.art)
                _gp = self.dc.get_rigid_body_pose(self._gt_body)
                _wx, _wy = float(_gp.p.x), float(_gp.p.y)
                _wyaw = euler_from_quaternion(
                    _gp.r.x, _gp.r.y, _gp.r.z, _gp.r.w)[2]
                # PhysX が一度でも発散すると真値姿勢が NaN になる。NaN は例外を
                # 出さないので下の except では捕まらず、そのまま _gt_ref に
                # 焼き付くと以降の odom も kinematic 目標も恒久的に NaN 化して
                # sim の再起動が要る状態になる。基準に採る前に必ず弾く。
                if not all(math.isfinite(_v) for _v in (_wx, _wy, _wyaw)):
                    raise ValueError(
                        '非有限の真値 odom: (%r, %r, %r)' % (_wx, _wy, _wyaw))
                if getattr(self, '_gt_ref', None) is None:
                    self._gt_ref = (_wx, _wy, _wyaw,
                                    self.odometry_estimator.pose.x,
                                    self.odometry_estimator.pose.y,
                                    self.odometry_estimator.pose.ang)
                _rwx, _rwy, _rwyaw, _rox, _roy, _royaw = self._gt_ref
                _ddx, _ddy = _wx - _rwx, _wy - _rwy
                _rc, _rs = math.cos(_rwyaw), math.sin(_rwyaw)
                _relx = _ddx * _rc + _ddy * _rs
                _rely = -_ddx * _rs + _ddy * _rc
                _relyaw = math.atan2(
                    math.sin(_wyaw - _rwyaw), math.cos(_wyaw - _rwyaw))
                _oc, _os = math.cos(_royaw), math.sin(_royaw)
                self.odometry_estimator.pose.x = _rox + _relx * _oc - _rely * _os
                self.odometry_estimator.pose.y = _roy + _relx * _os + _rely * _oc
                self.odometry_estimator.pose.ang = math.atan2(
                    math.sin(_royaw + _relyaw), math.cos(_royaw + _relyaw))
            except Exception as exc:
                # 真値が取れないフレームは前回の odom を据え置く。黙って捨てると
                # 発散に気付けないので一度だけ通知する。
                self._warn_once(
                    'gt-odom', '真値 odom の取り込みに失敗: %s' % exc)

        odom = Odometry()
        odom.header.stamp = self.get_ros_time(
            self.simulation_context.current_time)
        odom.header.frame_id = 'odom'
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
        base_trajectory_active_now = (
            self.odom_trajectory_action_server._action_goal is not None)
        base_trajectory_was_active = self._base_trajectory_was_active
        if base_trajectory_was_active and not base_trajectory_active_now:
            # cmd_vel received while MoveIt owned the base was ignored during
            # execution. Do not replay its two-second cache after the action;
            # only a fresh teleoperation message may release endpoint hold.
            self.cmd_vel_msg = None
            self.last_cmd_vel_time = 0.0
            # The action only succeeds below the endpoint velocity tolerance.
            # Clear the trajectory filter at that point so its residual command
            # cannot carry the base past the pose captured for endpoint hold.
            self._base_direct_cmd_prev = CartSpace()
            self._base_direct_applied_cmd = CartSpace()
            # Once a trajectory has succeeded inside the configured tolerance,
            # hold the accepted physical pose.  Continuing to pull toward the
            # mathematical endpoint caused millimetre-scale post-action creep.
            if self._base_hold_pose is not None:
                hold_x, hold_y, hold_yaw = self._base_hold_pose
                actual_pose = self.odometry_estimator.pose
                hold_errors = (
                    hold_x - actual_pose.x,
                    hold_y - actual_pose.y,
                    math.atan2(
                        math.sin(hold_yaw - actual_pose.ang),
                        math.cos(hold_yaw - actual_pose.ang),
                    ),
                )
                if max(abs(error) for error in hold_errors) <= (
                    _BASE_GOAL_TOLERANCE + 1e-6
                ):
                    self._base_hold_pose = (
                        actual_pose.x,
                        actual_pose.y,
                        actual_pose.ang,
                    )
                    self._reset_base_pose_integral()
        manual_cmd_active = (
            self.last_cmd_vel_time + _BASE_CMD_VEL_TIMEOUT
            > self.simulation_context.current_time
            and self.cmd_vel_msg is not None
            and (
                abs(self.cmd_vel_msg.linear.x) > 1e-6
                or abs(self.cmd_vel_msg.linear.y) > 1e-6
                or abs(self.cmd_vel_msg.angular.z) > 1e-6
            )
        )
        # 駐車ブレーキの目標 (odom 姿勢)。None ならブレーキ解除。
        # 「台車の軌道アクションが動いているか」ではなく「台車が今止まっている
        # べきか」で判断する。whole_body は腕だけ動かすときも台車コントローラへ
        # 「その場に留まれ」という軌道を送るため、前者で判定すると腕を振る間ずっと
        # ブレーキが解除され、反力で流されてしまう。
        brake_target = None
        brake_reason = ''
        # リセット直後は無条件で spawn 位置へ固定する (関節を戻す衝撃対策)。
        post_reset_hold = (
            self.simulation_context.current_time
            < getattr(self, '_post_reset_hold_until', 0.0))
        if base_trajectory_active_now:
            srv = self.odom_trajectory_action_server
            srv._odometry = self.odometry_estimator.pose
            # Keep the final trajectory target after all points are consumed.
            # The action server waits for odometry convergence; replacing this
            # target with the current pose made every under-tracked motion look
            # successful and stopped the base short of the MoveIt goal.
            # === 本物の OmniBaseController と同じ制御則 ===
            #   出力速度 = 目標速度(FF) + p_gain × 姿勢誤差(FB)   (odom 系)
            # FF: 軌道点が運ぶ「速度」をサンプリング(差分自作はスパイク暴発するので不可)。
            # FB: 目標位置 - 現在位置。向き誤差は必ず -pi..pi に wrap(π跨ぎ/角累積で
            #     ~2pi になり遠回りスピンするのを防ぐ)。
            _ffx, _ffy, _ffr = srv.sample_desired_velocity()
            _vx, _vy, _vr = (
                getattr(srv, '_velocity', None) or (0.0, 0.0, 0.0)
            )
            # Direct drive では whole-body の反力に追従負けしないよう、計測済みの
            # p_gain=2.0 を既定値にする。平面外の揺れは planar joint が拘束する。
            _dx = srv._joints['odom_x'] - self.odometry_estimator.pose.x
            _dy = srv._joints['odom_y'] - self.odometry_estimator.pose.y
            cmd.dot_x = (
                _ffx
                + _BASE_TRAJ_P_GAIN * _dx
                + _BASE_TRAJ_D_GAIN * (_ffx - _vx)
            )
            cmd.dot_y = (
                _ffy
                + _BASE_TRAJ_P_GAIN * _dy
                + _BASE_TRAJ_D_GAIN * (_ffy - _vy)
            )
            _dr = math.atan2(
                math.sin(
                    srv._joints['odom_t'] - self.odometry_estimator.pose.ang),
                math.cos(
                    srv._joints['odom_t'] - self.odometry_estimator.pose.ang),
            )
            cmd.dot_r = (
                _ffr
                + _BASE_TRAJ_P_GAIN * math.atan2(
                    math.sin(_dr), math.cos(_dr))
                + _BASE_TRAJ_D_GAIN * (_ffr - _vr)
            )
            # Stop only after the pose itself is inside the action tolerance.
            # A velocity deadband left a steady error of deadband / P-gain
            # (about 22 mm with the previous 0.01 / 0.45 settings).
            if abs(_ffx) < 1e-4 and abs(_dx) <= _BASE_CONTROL_DEADBAND:
                cmd.dot_x = -_BASE_TRAJ_D_GAIN * _vx
            if abs(_ffy) < 1e-4 and abs(_dy) <= _BASE_CONTROL_DEADBAND:
                cmd.dot_y = -_BASE_TRAJ_D_GAIN * _vy
            if abs(_ffr) < 1e-4 and abs(_dr) <= _BASE_CONTROL_DEADBAND:
                cmd.dot_r = -_BASE_TRAJ_D_GAIN * _vr

            # While following the time-parameterized path, FF+P avoids integral
            # windup.  At the final point, add a bounded integral term so the
            # static reaction from an extended arm cannot leave the base a
            # couple of centimetres short of the MoveIt endpoint.
            trajectory_done = (
                srv._action_point_index
                >= len(srv._action_goal.trajectory.points)
            )
            if trajectory_done:
                _ix, _iy, _ir = self._base_integral_command(
                    0.0 if abs(_dx) <= _BASE_CONTROL_DEADBAND else _dx,
                    0.0 if abs(_dy) <= _BASE_CONTROL_DEADBAND else _dy,
                    0.0 if abs(_dr) <= _BASE_CONTROL_DEADBAND else _dr,
                    dt,
                )
                cmd.dot_x += _ix
                cmd.dot_y += _iy
                cmd.dot_r += _ir
            else:
                self._reset_base_pose_integral()

            # 軌道自体が台車を動かそうとしていない (目標速度ほぼ 0) 区間で、
            # かつ目標付近にいるならブレーキで固定する。腕だけを動かす
            # whole_body 動作はこの条件に入り続けるので終始台車が動かない。
            if (
                trajectory_done
                and max(abs(_ffx), abs(_ffy), abs(_ffr)) < 1e-3
                and math.hypot(_dx, _dy) <= _BASE_BRAKE_ENGAGE_LINEAR
                and abs(_dr) <= _BASE_BRAKE_ENGAGE_ANGULAR
            ):
                brake_target = (
                    srv._joints['odom_x'],
                    srv._joints['odom_y'],
                    srv._joints['odom_t'],
                )
            else:
                brake_reason = (
                    '軌道追従中 ff=%.3f 誤差=%.3fm/%.3frad'
                    % (max(abs(_ffx), abs(_ffy), abs(_ffr)),
                       math.hypot(_dx, _dy), abs(_dr)))
            # Keep the last desired pose after the action completes.  Direct
            # drive has no wheel-ground traction, so releasing the chassis here
            # lets arm reaction forces move it away from the reached goal.
            self._base_hold_pose = (
                srv._joints['odom_x'],
                srv._joints['odom_y'],
                srv._joints['odom_t'],
            )
        elif manual_cmd_active:
            self._base_hold_pose = None  # 手動指令中はホールド解除
            brake_reason = 'cmd_vel 受信中'
            self._reset_base_pose_integral()
            ang = self.odometry_estimator.pose.ang + 0.5 * self.cmd_vel_msg.angular.z * dt
            cosr = math.cos(ang)
            sinr = math.sin(ang)
            cmd.dot_x = self.cmd_vel_msg.linear.x * cosr - self.cmd_vel_msg.linear.y * sinr
            cmd.dot_y = self.cmd_vel_msg.linear.x * sinr + self.cmd_vel_msg.linear.y * cosr
            cmd.dot_r = self.cmd_vel_msg.angular.z
        else:
            cmd.dot_x = 0.0
            cmd.dot_y = 0.0
            cmd.dot_r = 0.0
            if _BASE_DIRECT_DRIVE:
                # Keep holding the planned endpoint.  Capturing the current
                # pose here makes a failed or slightly under-tracked action
                # permanently stop short.
                if self._base_hold_pose is None:
                    self._base_hold_pose = (
                        self.odometry_estimator.pose.x,
                        self.odometry_estimator.pose.y,
                        self.odometry_estimator.pose.ang,
                    )
                    self._reset_base_pose_integral()
                if base_trajectory_was_active:
                    print(
                        '[base-hold] target=(%.4f, %.4f, %.4f)'
                        % self._base_hold_pose,
                        flush=True,
                    )
                hx, hy, ht = self._base_hold_pose
                hold_dx = hx - self.odometry_estimator.pose.x
                hold_dy = hy - self.odometry_estimator.pose.y
                hold_dr = math.atan2(
                    math.sin(ht - self.odometry_estimator.pose.ang),
                    math.cos(ht - self.odometry_estimator.pose.ang),
                )
                # 常にブレーキで保持する。遠い場合も目標を毎回いまの位置へ
                # 付け替えると、外力で押されるたびに保持位置がずれていき、
                # 台車が滑り続けてしまう (reset 後に実測)。保持位置は固定し、
                # そこへ creep 速度でゆっくり戻す。
                brake_target = self._base_hold_pose
        # 軌道追従していない間(cmd_vel/静止)は /omni_base_controller/state の actual(=_joints)を
        # 現在の真値odomで更新し続ける。これをしないと計画が読む台車位置が古い(0)ままで、
        # 台車が cmd_vel(nav)で動いた後の whole_body 計画が間違った位置から立ち、台車が暴れる。
        if self.odom_trajectory_action_server._action_goal is None:
            _sj = self.odom_trajectory_action_server._joints
            _sj['odom_x'] = self.odometry_estimator.pose.x
            _sj['odom_y'] = self.odometry_estimator.pose.y
            _sj['odom_t'] = self.odometry_estimator.pose.ang

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

        _base_trajectory_active = base_trajectory_active_now
        self._base_trajectory_was_active = _base_trajectory_active
        _base_idle = (
            not _base_trajectory_active and
            abs(cmd.dot_x) < 1e-6 and
            abs(cmd.dot_y) < 1e-6 and
            abs(cmd.dot_r) < 1e-6
        )

        if post_reset_hold:
            # リセット直後にブレーキを掛けると、テレポート直後の拘束状態と
            # バネ目標が噛み合わず台車が飛ばされる (実測で毎回同じ位置へ 2m)。
            # 落ち着くまではブレーキを解除し、ドライブ目標を実位置に追従させて
            # 力が出ない状態にしておく。
            # 全関節を一瞬でホーム姿勢へ戻す衝撃で台車が動くので、その間は
            # 誤差の大小によらず spawn 位置へ固定し続ける。
            brake_target = (0.0, 0.0, 0.0)
            self._base_hold_pose = (0.0, 0.0, 0.0)
            cmd = CartSpace()
        if _BASE_DIRECT_DRIVE and _BASE_BRAKE_ENABLED and brake_target is not None:
            # --- 駐車ブレーキ: 姿勢を固定して腕の反力に流されないようにする ---
            self._base_direct_cmd_prev = CartSpace()
            self._base_direct_applied_cmd = CartSpace()
            self._base_direct_pose = None
            self._base_kinematic_velocity = (0.0, 0.0, 0.0)
            self._apply_base_brake(brake_target, dt)
            if _BASE_KINEMATIC_DRIVE:
                # 停止時だけ物理ブレーキへ戻すと、移動中に消したソルバー競合が
                # 再発し、停止後に数十 cm 揺り戻される。ブレーキの creep で求めた
                # pin を articulation 姿勢にも反映して、停止保持も決定論的にする。
                self._set_kinematic_base_pose(self._base_brake_pin)
            self.dc.set_dof_velocity_target(self.left_wheel_ptr, 0.0)
            self.dc.set_dof_velocity_target(self.right_wheel_ptr, 0.0)
            self.dc.set_dof_velocity_target(self.roll_ptr, 0.0)
        elif _BASE_DIRECT_DRIVE:
            self._release_base_brake(brake_reason)
            direct_cmd = getattr(self, '_base_direct_cmd_prev', CartSpace())
            if dt > 0.0:
                alpha = min(1.0, dt / (_BASE_CMD_TAU + dt))
                filtered = CartSpace()
                filtered.dot_x = direct_cmd.dot_x + alpha * (
                    cmd.dot_x - direct_cmd.dot_x)
                filtered.dot_y = direct_cmd.dot_y + alpha * (
                    cmd.dot_y - direct_cmd.dot_y)
                filtered.dot_r = direct_cmd.dot_r + alpha * (
                    cmd.dot_r - direct_cmd.dot_r)

                delta_x = filtered.dot_x - direct_cmd.dot_x
                delta_y = filtered.dot_y - direct_cmd.dot_y
                delta_r = filtered.dot_r - direct_cmd.dot_r
                accel_scale = 1.0
                delta_linear = math.hypot(delta_x, delta_y)
                if _BASE_LINEAR_ACCEL_LIMIT > 0.0 and delta_linear > 0.0:
                    accel_scale = min(
                        accel_scale,
                        _BASE_LINEAR_ACCEL_LIMIT * dt / delta_linear,
                    )
                if _BASE_ANGULAR_ACCEL_LIMIT > 0.0 and abs(delta_r) > 0.0:
                    accel_scale = min(
                        accel_scale,
                        _BASE_ANGULAR_ACCEL_LIMIT * dt / abs(delta_r),
                    )

                smoothed = CartSpace()
                smoothed.dot_x = direct_cmd.dot_x + accel_scale * delta_x
                smoothed.dot_y = direct_cmd.dot_y + accel_scale * delta_y
                smoothed.dot_r = direct_cmd.dot_r + accel_scale * delta_r
                if (
                    abs(cmd.dot_x) < 1e-6
                    and abs(cmd.dot_y) < 1e-6
                    and abs(cmd.dot_r) < 1e-6
                    and max(
                        abs(smoothed.dot_x),
                        abs(smoothed.dot_y),
                        abs(smoothed.dot_r),
                    ) < 1e-3
                ):
                    smoothed = CartSpace()
                direct_cmd = smoothed
                self._base_direct_cmd_prev = smoothed

            kinematic_applied = (
                _BASE_KINEMATIC_DRIVE
                and self._apply_kinematic_base_motion(direct_cmd, dt)
            )
            if kinematic_applied:
                # 指令値ではなく、衝突クランプ後に実現できた速度を返す。
                # ここは軌道追従の D 項 (odom_trajectory_action_server._velocity)
                # と次ステップの /odom twist の両方の元になるので、壁で止まった
                # ことがナビゲーション側から観測できるようになる。
                _kv = getattr(
                    self, '_base_kinematic_velocity', (0.0, 0.0, 0.0))
                abs_dot_x = float(_kv[0])
                abs_dot_y = float(_kv[1])
                cartesian_param_.dot_r = float(_kv[2])
                # ここで articulation root の速度を書いてはいけない。root は
                # FixedJoint で anchor に溶接されているので、拘束が要求する速度は
                # 常に anchor の速度そのもの。別の値を書くとソルバーが毎ステップ
                # それを打ち消し、その揺れが arm_flex まで伝わる。anchor と root の
                # 速度整合は、anchor を kinematic target で動かして PhysX 自身に
                # 速度を導出させることで成立させる
                # (BASE_KINEMATIC_ANCHOR_MODE=usd)。

            # 従来経路。決定論的更新が無効、または Tensor API が失敗した場合だけ
            # 剛体速度と位置ドライブを併用する。
            # BASE_VELOCITY_WRITE=0 にすると速度書き込みを省き、平面ジョイントの
            # ドライブ単独で走らせる (二重制御の切り分け用。既定は従来どおり)。
            # BASE_ROT_NO_VELOCITY_WRITE=1 は純回転のときだけ書き込みを省く。
            # 純回転では速度書き込みと位置ドライブの競合で並進が 3 倍に増える
            # (実測 0.31m -> 0.93m)。直進は速度書き込みが無いと成立しないため、
            # 回転のときだけ角度ドライブ単独にする。
            _pure_rotation = (
                _BASE_ROT_NO_VELOCITY_WRITE
                and abs(direct_cmd.dot_x) < 1e-3
                and abs(direct_cmd.dot_y) < 1e-3
                and abs(direct_cmd.dot_r) > 1e-3
            )
            if _BASE_DIAG and _BASE_ROT_NO_VELOCITY_WRITE:
                if _pure_rotation != getattr(self, '_pure_rot_prev', None):
                    self._pure_rot_toggle = (
                        getattr(self, '_pure_rot_toggle', 0) + 1)
                    print('[rot-switch] %s (%d回目) cmd=(%.4f, %.4f, %.4f)'
                          % ('純回転' if _pure_rotation else '通常',
                             self._pure_rot_toggle, direct_cmd.dot_x,
                             direct_cmd.dot_y, direct_cmd.dot_r), flush=True)
                    self._pure_rot_prev = _pure_rotation
            if (not kinematic_applied
                    and _BASE_VELOCITY_WRITE and not _pure_rotation):
                root_velocity = self.robots.get_velocities()[0]
                self.robots.set_velocities(np.array([[
                    direct_cmd.dot_x,
                    direct_cmd.dot_y,
                    float(root_velocity[2]),
                    0.0,
                    0.0,
                    direct_cmd.dot_r,
                ]], dtype=np.float32))
            self._base_direct_applied_cmd = direct_cmd
            # 平面ジョイントのドライブを「位置サーボ」として使う。目標は指令速度を
            # 積分した設定値 (setpoint)。現在位置に追従させるとバネが動きを
            # 打ち消してしまい走行できない (実測: 0.5m 指令で 0.12m しか進まず)。
            if not kinematic_applied:
                self._advance_base_setpoint(direct_cmd, dt)
            # ドライブの減衰項は速度に比例した抵抗になる (速度目標は PhysX に
            # 反映されないため実測で確認)。抵抗と釣り合う分だけ目標位置を
            # 先出しして打ち消す。lead = 減衰/剛性 x 指令速度。
            lead = (
                _BASE_BRAKE_C / _BASE_BRAKE_K if _BASE_BRAKE_K > 0.0 else 0.0)
            lead_ang = (
                _BASE_BRAKE_ANGULAR_C / _BASE_BRAKE_ANGULAR_K
                if _BASE_BRAKE_ANGULAR_K > 0.0 else 0.0)
            if not kinematic_applied:
                self._set_joint_brake_target((
                    self._base_setpoint[0] + lead * direct_cmd.dot_x,
                    self._base_setpoint[1] + lead * direct_cmd.dot_y,
                    self._base_setpoint[2] + lead_ang * direct_cmd.dot_r,
                ))
            if _BASE_DIAG and max(
                abs(direct_cmd.dot_x), abs(direct_cmd.dot_y),
                abs(direct_cmd.dot_r)) > 1e-3:
                self._base_move_diag_count = (
                    getattr(self, '_base_move_diag_count', 0) + 1)
                if self._base_move_diag_count % 20 == 1:
                    try:
                        _v = self.robots.get_velocities()[0]
                        _vx, _vy = float(_v[0]), float(_v[1])
                    except Exception:
                        _vx = _vy = 0.0
                    print(
                        '[move-diag] cmd=(%.3f, %.3f) 実速度=(%.3f, %.3f) '
                        'setpoint先行=(%.4f, %.4f)'
                        % (direct_cmd.dot_x, direct_cmd.dot_y, _vx, _vy,
                           self._base_setpoint[0]
                           - self.odometry_estimator.pose.x,
                           self._base_setpoint[1]
                           - self.odometry_estimator.pose.y),
                        flush=True,
                    )
            self.dc.set_dof_velocity_target(self.left_wheel_ptr, 0.0)
            self.dc.set_dof_velocity_target(self.right_wheel_ptr, 0.0)
            self.dc.set_dof_velocity_target(self.roll_ptr, 0.0)
        elif dt > 0.0:
            # MoveIt の仮想 odom 軌道は点列境界やFB補正で速度targetが小刻みに変わる。
            # そのまま PhysX の車輪/ステア velocity target へ入れると到達はしても
            # 加減速の角で車体がぐらつくため、実行直前に一次遅れ + 加速度制限を掛ける。
            # 停止時もこの経路を通す。以前は idle になった一フレームで速度目標を
            # 0へ落としていたため、強い車輪ドライブほど停止衝撃が大きくなっていた。
            _prev = getattr(self, '_base_cmd_prev', JointSpace())
            _alpha = min(1.0, dt / (_BASE_CMD_TAU + dt))

            # Apply the same acceleration scale to all three joint axes.
            # Independent clipping (previous implementation) changed the
            # wheel/steer ratio required by TwinCasterDrive, so a lateral
            # command first drove strongly in the wrong direction and shook
            # the chassis.  This is the same principle used by the real HSR
            # controller's coupled acceleration limiter.
            _filtered = JointSpace()
            _filtered.vel_wheel_l = _prev.vel_wheel_l + _alpha * (
                jcmd.vel_wheel_l - _prev.vel_wheel_l)
            _filtered.vel_wheel_r = _prev.vel_wheel_r + _alpha * (
                jcmd.vel_wheel_r - _prev.vel_wheel_r)
            _filtered.vel_steer = _prev.vel_steer + _alpha * (
                jcmd.vel_steer - _prev.vel_steer)
            _deltas_and_limits = (
                (_filtered.vel_wheel_l - _prev.vel_wheel_l,
                 _BASE_WHEEL_ACCEL_LIMIT),
                (_filtered.vel_wheel_r - _prev.vel_wheel_r,
                 _BASE_WHEEL_ACCEL_LIMIT),
                (_filtered.vel_steer - _prev.vel_steer,
                 _BASE_STEER_ACCEL_LIMIT),
            )
            _accel_scale = 1.0
            for _delta, _limit in _deltas_and_limits:
                if abs(_delta) > 0.0:
                    _accel_scale = min(
                        _accel_scale, _limit * dt / abs(_delta))
            _smoothed = JointSpace()
            _smoothed.vel_wheel_l = _prev.vel_wheel_l + _accel_scale * (
                _filtered.vel_wheel_l - _prev.vel_wheel_l)
            _smoothed.vel_wheel_r = _prev.vel_wheel_r + _accel_scale * (
                _filtered.vel_wheel_r - _prev.vel_wheel_r)
            _smoothed.vel_steer = _prev.vel_steer + _accel_scale * (
                _filtered.vel_steer - _prev.vel_steer)
            if _base_idle and max(
                abs(_smoothed.vel_wheel_l),
                abs(_smoothed.vel_wheel_r),
                abs(_smoothed.vel_steer),
            ) < 1e-3:
                _smoothed = JointSpace()
            jcmd = _smoothed
            self._base_cmd_prev = _smoothed

        if not _BASE_DIRECT_DRIVE:
            # Do not switch DOF stiffness at runtime.  Isaac Sim 4.5 rebuilds
            # the articulation for set_dof_properties(), invalidating cached
            # dynamic-control handles.
            self.dc.set_dof_velocity_target(
                self.left_wheel_ptr, jcmd.vel_wheel_l)
            self.dc.set_dof_velocity_target(
                self.right_wheel_ptr, jcmd.vel_wheel_r)
            self.dc.set_dof_velocity_target(self.roll_ptr, jcmd.vel_steer)

        force_readings = self.robots.get_measured_joint_forces(
            joint_indices=[
                self.robots._metadata.joint_indices['wrist_ft_sensor_frame_joint'] + 1]
        )
        # 物理ビュー/FTセンサが未準備のフレームでは None が返ることがある(特に world
        # 差し替え/reset 直後)。そのまま force_readings[0]... を実行すると毎フレーム
        # TypeError で step() 全体が落ち、以降の action server(腕/台車/グリッパの軌道実行)が
        # 一切動かず把持が進まない。ゼロ読みでフォールバックして step() を継続させる
        # (センサ復帰後は実値に戻る)。
        ft_valid = force_readings is not None
        if not ft_valid:
            force_readings = [[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]]
            if not getattr(self, '_ft_none_warned', False):
                self._ft_none_warned = True
                print('[hsr] get_measured_joint_forces returned None; FT/wrench をゼロで代用',
                      flush=True)
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
        # 注意: None フォールバックの偽ゼロを baseline に取り込むと、ベースラインが0に
        # 初期化/引き寄せられ、実センサ復帰後しばらく補償後 wrench に重力ぶんが誤って残る
        # (接触検知を汚染する)。そのため初期化・EMA更新・comp配信は実FT読みのときだけ行う。
        if ft_valid:
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
        self.odom_trajectory_action_server._velocity = (
            abs_dot_x,
            abs_dot_y,
            cartesian_param_.dot_r,
        )
        self.odom_trajectory_action_server.step(dt=dt)
        self.gripper_trajectory_action_server.step(dt=dt)
        self.gripper_apply_force_action_server.step(dt=dt)
        self.gripper_command_action_server.step(dt=dt)

        self.prev_time = self.simulation_context.current_time
