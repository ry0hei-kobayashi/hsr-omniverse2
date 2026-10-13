"""物理パラメータ調整 (質量・慣性・ジョイント・ドライブ・摩擦) (hsr.py から分離した PhysicsTuningMixin)。

class hsr に多重継承で混ぜる。メソッドは self の属性だけを使い、状態は hsr.__init__ が持つ。
"""
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
from hsr_omniverse_sim.robot.hsr.params import (_ARM_FLEX_MAX_FORCE, _BASE_BRAKE_ANGULAR_C, _BASE_BRAKE_ANGULAR_K, _BASE_BRAKE_C, _BASE_BRAKE_K, _BASE_BRAKE_MAX_FORCE, _BASE_BRAKE_MAX_TORQUE, _BASE_DIRECT_DRIVE, _BASE_DIRECT_JOINT_DAMPING, _BASE_DIRECT_JOINT_MAX_FORCE, _BASE_DIRECT_ROOT_DAMPING, _BASE_JOINT_BRAKE, _BASE_KINEMATIC_DRIVE, _BASE_PASSIVE_WHEEL_DAMPING, _BASE_STEER_DRIVE_DAMPING, _BASE_STEER_DRIVE_MAX_FORCE, _BASE_WHEEL_DRIVE_DAMPING, _BASE_WHEEL_DRIVE_MAX_FORCE)
from hsr_omniverse_sim.robot.hsr.quat import euler_from_quaternion


class PhysicsTuningMixin:
    def _bind_physics_material(self, material_path, prim_paths):
        """Bind a USD material for PhysX without the removed BindMaterialExt command."""
        current_stage = stage.get_current_stage()
        material_prim = current_stage.GetPrimAtPath(material_path)
        if not material_prim or not material_prim.IsValid():
            raise RuntimeError('missing physics material: %s' % material_path)
        material = UsdShade.Material(material_prim)

        bound = []
        for prim_path in prim_paths:
            prim = current_stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                raise RuntimeError('missing collision prim: %s' % prim_path)
            binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
            binding_api.Bind(
                material,
                UsdShade.Tokens.weakerThanDescendants,
                'physics',
            )
            bound.append(prim_path)
        return bound
    def _set_base_root_mass(self):
        # base_footprint is an articulation body without a collider.  Its USD
        # mass=0 makes PhysX substitute a tiny sphere inertia, which gives the
        # mobile base an ill-conditioned root during arm/base motion.  Direct
        # drive can only set state on this root, so give it comparable inertia
        # to base_link instead of leaving a 1 kg body fixed to a 50 kg chassis.
        root_mass = 50.0 if _BASE_DIRECT_DRIVE else 1.0
        root_inertia = (
            Gf.Vec3f(0.4, 0.4, 0.2)
            if _BASE_DIRECT_DRIVE
            else Gf.Vec3f(0.02, 0.02, 0.02)
        )
        root_damping = (
            _BASE_DIRECT_ROOT_DAMPING if _BASE_DIRECT_DRIVE else 2.0)
        root_path = self.stage_path + self.prefix + '/base_footprint'
        root_prim = stage.get_current_stage().GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            raise RuntimeError('missing articulation root: %s' % root_path)
        mass_api = UsdPhysics.MassAPI.Apply(root_prim)
        mass_api.CreateMassAttr().Set(root_mass)
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        mass_api.CreateDiagonalInertiaAttr().Set(root_inertia)
        rigid_body_api = PhysxSchema.PhysxRigidBodyAPI(root_prim)
        if not rigid_body_api:
            rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(root_prim)
        rigid_body_api.CreateLinearDampingAttr().Set(root_damping)
        rigid_body_api.CreateAngularDampingAttr().Set(root_damping)
        print(
            '[base-physics] base_footprint mass=%.1fkg '
            'inertia=%s damping=%.1f' %
            (root_mass, tuple(root_inertia), root_damping),
            flush=True,
        )
    def _add_planar_root_joint(self):
        if not _BASE_DIRECT_DRIVE:
            return

        # articulation root を毎フレーム直接テレポートすると、内部関節と外部 D6
        # 拘束の解決が衝突して物理が暴れる。kinematic 時は「衝突形状を持たない
        # kinematic anchor」を作り、base_footprint を FixedJoint で結ぶ。移動するのは
        # anchor だけなので、ロボット内部の物理を保ったまま決定論的に台車を拘束できる。
        if _BASE_KINEMATIC_DRIVE:
            current_stage = stage.get_current_stage()
            root_path = self.stage_path + self.prefix + '/base_footprint'
            root_prim = current_stage.GetPrimAtPath(root_path)
            if not root_prim or not root_prim.IsValid():
                raise RuntimeError('missing kinematic base body: %s' % root_path)
            root_transform = UsdGeom.Xformable(
                root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            root_translation = root_transform.ExtractTranslation()
            root_quat = Gf.Transform(root_transform).GetRotation().GetQuat()
            root_imaginary = root_quat.GetImaginary()
            root_yaw = euler_from_quaternion(
                float(root_imaginary[0]), float(root_imaginary[1]),
                float(root_imaginary[2]), float(root_quat.GetReal()))[2]

            anchor_path = self.stage_path + '/hsr_base_kinematic_anchor'
            anchor = UsdGeom.Xform.Define(current_stage, anchor_path)
            anchor_xform = UsdGeom.XformCommonAPI(anchor)
            anchor_xform.SetTranslate(root_translation)
            anchor_xform.SetRotate(
                Gf.Vec3f(0.0, 0.0, math.degrees(root_yaw)),
                UsdGeom.XformCommonAPI.RotationOrderXYZ,
            )
            anchor_body = UsdPhysics.RigidBodyAPI.Apply(anchor.GetPrim())
            anchor_body.CreateKinematicEnabledAttr().Set(True)

            joint_path = self.stage_path + '/hsr_kinematic_base_joint'
            joint = UsdPhysics.FixedJoint.Define(current_stage, joint_path)
            joint.CreateBody0Rel().SetTargets([Sdf.Path(anchor_path)])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(root_path)])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
            joint.CreateExcludeFromArticulationAttr().Set(True)
            joint.CreateBreakForceAttr().Set(1e20)
            joint.CreateBreakTorqueAttr().Set(1e20)

            self._base_kinematic_anchor_path = anchor_path
            self._base_kinematic_anchor = None
            self._base_brake_drives = {}
            self._planar_frame0 = None
            print(
                '[base-physics] kinematic drive: fixed anchor=%s' % anchor_path,
                flush=True,
            )
            return

        current_stage = stage.get_current_stage()
        root_path = self.stage_path + self.prefix + '/base_footprint'
        root_prim = current_stage.GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            raise RuntimeError('missing planar-joint body: %s' % root_path)

        root_transform = UsdGeom.Xformable(
            root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        root_translation = root_transform.ExtractTranslation()
        root_quat = Gf.Transform(root_transform).GetRotation().GetQuat()
        root_imaginary = root_quat.GetImaginary()
        root_orientation = Gf.Quatf(
            float(root_quat.GetReal()),
            Gf.Vec3f(
                float(root_imaginary[0]),
                float(root_imaginary[1]),
                float(root_imaginary[2]),
            ),
        )

        joint_path = self.stage_path + '/hsr_planar_world_joint'
        joint = UsdPhysics.Joint.Define(current_stage, joint_path)
        joint.CreateBody1Rel().SetTargets([Sdf.Path(root_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(
            float(root_translation[0]),
            float(root_translation[1]),
            float(root_translation[2]),
        ))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(root_orientation)
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        joint.CreateExcludeFromArticulationAttr().Set(True)
        joint.CreateBreakForceAttr().Set(1e20)
        joint.CreateBreakTorqueAttr().Set(1e20)

        # A D6 axis is locked when low > high.  Leave transX/transY/rotZ
        # without LimitAPI so the mobile base keeps its planar degrees of
        # freedom while vertical, roll, and pitch reactions are solved as
        # constraints instead of being cancelled one frame later.
        for axis in ('transZ', 'rotX', 'rotY'):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            limit.CreateLowAttr().Set(1.0)
            limit.CreateHighAttr().Set(-1.0)

        # 駐車ブレーキ用ドライブ。台車を保持する力を「拘束ソルバー内」で出せる
        # 唯一の経路。stiffness は起動時(物理構築前)に入れておく — 実行中の
        # USD 属性変更は PhysX に伝わらない場合があるため。保持位置は
        # targetPosition を毎ステップ更新して指定する。走行中は現在位置を
        # 目標にし続ければバネ力はほぼ 0 になり、従来の速度制御を妨げない。
        # joint frame0 は spawn 姿勢なので DOF 値 = odom 座標と一致する。
        self._base_brake_drives = {}
        # kinematic drive は停止中も articulation 姿勢を直接保持する。そこでさらに
        # 20万 N/m の位置ドライブを有効にすると、姿勢更新直後の物理ステップで
        # 引き戻しが起きる。kinematic 時は D6 の z/roll/pitch 拘束だけを残す。
        if _BASE_JOINT_BRAKE and not _BASE_KINEMATIC_DRIVE:
            for axis in ('transX', 'transY', 'rotZ'):
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
                drive.CreateTypeAttr().Set('force')
                if axis == 'rotZ':
                    # USD の回転ドライブは度単位。
                    drive.CreateStiffnessAttr().Set(
                        _BASE_BRAKE_ANGULAR_K * math.pi / 180.0)
                    drive.CreateDampingAttr().Set(
                        _BASE_BRAKE_ANGULAR_C * math.pi / 180.0)
                    drive.CreateMaxForceAttr().Set(_BASE_BRAKE_MAX_TORQUE)
                else:
                    drive.CreateStiffnessAttr().Set(_BASE_BRAKE_K)
                    drive.CreateDampingAttr().Set(_BASE_BRAKE_C)
                    drive.CreateMaxForceAttr().Set(_BASE_BRAKE_MAX_FORCE)
                drive.CreateTargetPositionAttr().Set(0.0)
                self._base_brake_drives[axis] = drive
        # ドライブの DOF 値は「この frame0 からの変位」。odom の原点は起動直後の
        # わずかな移動でここからずれるので、その差を後で補正する必要がある
        # (補正しないとサーボの指令が常にずれ、台車が目標に到達できない)。
        self._planar_frame0 = (
            float(root_translation[0]),
            float(root_translation[1]),
            float(euler_from_quaternion(
                float(root_orientation.GetImaginary()[0]),
                float(root_orientation.GetImaginary()[1]),
                float(root_orientation.GetImaginary()[2]),
                float(root_orientation.GetReal()))[2]),
        )
        print(
            '[base-physics] planar joint: free=(x,y,yaw) '
            'locked=(z,roll,pitch) joint_brake=%s frame0=(%.4f, %.4f, %.4f) '
            'k=%.0f d=%.0f'
            % ('on' if self._base_brake_drives else 'off',
               float(root_translation[0]), float(root_translation[1]),
               float(root_translation[2]),
               _BASE_BRAKE_K, _BASE_BRAKE_C),
            flush=True,
        )
    def _tune_articulation_solver(self):
        root_path = self.stage_path + self.prefix
        root_prim = stage.get_current_stage().GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            raise RuntimeError('missing articulation prim: %s' % root_path)
        articulation_api = PhysxSchema.PhysxArticulationAPI(root_prim)
        if not articulation_api:
            articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(
                root_prim)

        # The imported asset requests 16 velocity iterations.  Isaac Sim 4.5
        # explicitly warns that TGS behavior changed above four iterations;
        # for the offset caster it over-solves alternating tire/steer impulses
        # and produces wheel-speed spikes well beyond the commanded limit.
        articulation_api.CreateSolverVelocityIterationCountAttr().Set(4)
        print(
            '[base-physics] articulation solver velocity iterations=4',
            flush=True,
        )
    def _tune_base_joint_dynamics(self):
        joint_specs = {
            'base_l_drive_wheel_joint': (8.5, 0.015),
            'base_r_drive_wheel_joint': (8.5, 0.015),
            'base_roll_joint': (1.8, 0.05),
        }
        found = set()
        for prim in stage.get_current_stage().Traverse():
            name = prim.GetName()
            if name not in joint_specs:
                continue
            max_velocity, armature = joint_specs[name]
            joint_api = PhysxSchema.PhysxJointAPI(prim)
            if not joint_api:
                joint_api = PhysxSchema.PhysxJointAPI.Apply(prim)
            # Angular USD attributes use degrees while dynamic-control uses
            # radians.  The imported 1191.75 deg/s cap allowed contact impulses
            # to drive wheels to 20.8 rad/s despite the 8.5 rad/s command cap.
            joint_api.CreateMaxJointVelocityAttr().Set(
                math.degrees(max_velocity))
            joint_api.CreateArmatureAttr().Set(armature)
            found.add(name)
            print(
                '[base-joint] %s maxVel=%.3g rad/s armature=%.3g'
                % (name, max_velocity, armature),
                flush=True,
            )
        missing = set(joint_specs) - found
        if missing:
            raise RuntimeError(
                'base joint prims not found: %s' % sorted(missing))
    def _fix_arm_link_inertia(self):
        # hsrb4s.usd was generated from an old test URDF that has arm_flex
        # ixx=7.528.  The current hsrb_description URDF has ixx=0.007528;
        # the 1000x error makes the flex drive saturate and oscillate.
        link_path = self.stage_path + self.prefix + '/arm_flex_link'
        link_prim = stage.get_current_stage().GetPrimAtPath(link_path)
        if not link_prim or not link_prim.IsValid():
            raise RuntimeError('missing arm link prim: %s' % link_path)
        mass_api = UsdPhysics.MassAPI.Apply(link_prim)
        mass_api.CreateDiagonalInertiaAttr().Set(
            Gf.Vec3f(0.007528, 0.007102, 0.001552))
        mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0))
        print(
            '[arm-physics] arm_flex_link inertia='
            '(0.007528,0.007102,0.001552)kg*m^2',
            flush=True,
        )
    def _tune_arm_joint_dynamics(self):
        # The imported model has zero reflected motor inertia on every joint.
        # Once arm_flex's 1000x inertia typo is corrected, the light wrist
        # bodies otherwise respond to a one-step drive impulse too quickly for
        # the 60 Hz controller.  Small armatures keep the inertia ratios
        # well-conditioned without changing link mass or gravity torque.
        armatures = {
            'arm_flex_joint': 0.05,
            'arm_roll_joint': 0.02,
            'wrist_flex_joint': 0.01,
            'wrist_roll_joint': 0.01,
        }
        found = set()
        for prim in stage.get_current_stage().Traverse():
            name = prim.GetName()
            if name not in armatures:
                continue
            joint_api = PhysxSchema.PhysxJointAPI(prim)
            if not joint_api:
                joint_api = PhysxSchema.PhysxJointAPI.Apply(prim)
            joint_api.CreateArmatureAttr().Set(armatures[name])
            found.add(name)
            print(
                '[arm-joint] %s armature=%.3g'
                % (name, armatures[name]),
                flush=True,
            )
        missing = set(armatures) - found
        if missing:
            raise RuntimeError(
                'arm joint prims not found: %s' % sorted(missing))
    def _tune_arm_drives(self):
        # The imported USD uses generic gains (k=1e7, d=1e5) and force limits
        # of 18 kN / 6 kNm.  Angular USD gains are torque/degree, while the
        # controller and values below use radians.  Convert explicitly;
        # assigning the per-radian values directly made the effective gains
        # 57.3x too high and turned the position drive into a force limiter.
        drive_specs = {
            'arm_lift_joint': ('linear', 8000.0, 1000.0, 300.0),
            'torso_lift_joint': ('linear', 8000.0, 1000.0, 300.0),
            # k=500 だと腕を水平近くまで伸ばしたとき重力で 0.02〜0.04 rad
            # (先端で 2〜3cm) 垂れ、目標許容 0.03 を超えて whole_body 動作が
            # 「目標未達」で中断していた。剛性を上げてたわみを 1/2 以下にする。
            # 台車は平面ジョイントのドライブで固定されるので、反力が増えても
            # 車体が揺すられることはない。
            'arm_flex_joint': ('angular', 1200.0, 200.0, _ARM_FLEX_MAX_FORCE),
            'arm_roll_joint': ('angular', 200.0, 30.0, 100.0),
            'wrist_flex_joint': ('angular', 200.0, 30.0, 100.0),
            'wrist_roll_joint': ('angular', 100.0, 20.0, 100.0),
        }
        found = set()
        for prim in stage.get_current_stage().Traverse():
            name = prim.GetName()
            if name not in drive_specs:
                continue
            drive_type, stiffness_rad, damping_rad, max_force = (
                drive_specs[name])
            drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
            if not drive:
                raise RuntimeError(
                    'missing %s drive for %s' % (drive_type, prim.GetPath()))
            gain_scale = math.pi / 180.0 if drive_type == 'angular' else 1.0
            stiffness = stiffness_rad * gain_scale
            damping = damping_rad * gain_scale
            drive.GetStiffnessAttr().Set(stiffness)
            drive.GetDampingAttr().Set(damping)
            drive.GetMaxForceAttr().Set(max_force)
            drive.GetTargetVelocityAttr().Set(0.0)
            drive.GetTypeAttr().Set('force')
            found.add(name)
            print(
                '[arm-drive] %s k=%.3g/rad d=%.3g/rad '
                '(USD k=%.3g d=%.3g) maxF=%.1f'
                % (
                    name,
                    stiffness_rad,
                    damping_rad,
                    stiffness,
                    damping,
                    max_force,
                ),
                flush=True,
            )
        missing = set(drive_specs) - found
        if missing:
            raise RuntimeError(
                'arm drive prims not found: %s' % sorted(missing))
    def set_base_joint_and_material(self) -> None:
        self._set_base_root_mass()
        self._add_planar_root_joint()
        self._tune_articulation_solver()
        self._tune_base_joint_dynamics()
        self._fix_arm_link_inertia()
        self._tune_arm_joint_dynamics()
        self._tune_arm_drives()

        caster_material = PhysicsMaterial(
            prim_path='/Caster',
            static_friction=0.0,
            dynamic_friction=0.0,
        )
        caster_api = PhysxSchema.PhysxMaterialAPI.Apply(
            stage.get_current_stage().GetPrimAtPath('/Caster'))
        caster_api.CreateFrictionCombineModeAttr().Set('min')
        caster_paths = [
            self.stage_path + self.prefix + l
            for l in (
                '/base_l_passive_wheel_z_link/collisions',
                '/base_r_passive_wheel_z_link/collisions',
            )
        ]
        self._bind_physics_material('/Caster', caster_paths)

        # Direct mode is an ideal planar actuator for MoveIt whole-body tests.
        # Removing longitudinal tire friction prevents the unactuated
        # twin-caster joints from feeding steering impulses back into the
        # 70 kg chassis.  Normal contact remains active, so the base is still
        # supported by the floor and participates in vertical collisions.
        tire_static_friction = 0.0 if _BASE_DIRECT_DRIVE else 1.0
        tire_dynamic_friction = 0.0 if _BASE_DIRECT_DRIVE else 0.8
        tire_material = PhysicsMaterial(
            prim_path='/Tire',
            static_friction=tire_static_friction,
            dynamic_friction=tire_dynamic_friction,
        )
        tire_api = PhysxSchema.PhysxMaterialAPI.Apply(
            stage.get_current_stage().GetPrimAtPath('/Tire'))
        tire_api.CreateFrictionCombineModeAttr().Set(
            'min' if _BASE_DIRECT_DRIVE else 'max')
        tire_paths = [
            self.stage_path + self.prefix + l
            for l in (
                '/base_l_drive_wheel_link/collisions',
                '/base_r_drive_wheel_link/collisions',
            )
        ]
        self._bind_physics_material('/Tire', tire_paths)
        print(
            '[base-physics] mode=%s caster friction=0 tire friction=(%.1f,%.1f)'
            % (
                'direct' if _BASE_DIRECT_DRIVE else 'wheel',
                tire_static_friction,
                tire_dynamic_friction,
            ),
            flush=True,
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
        _finger_paths = []
        for _p in _st.Traverse():
            _ps = str(_p.GetPath())
            if _ps.startswith(_root) and 'hand_' in _ps and _ps.endswith('/collisions'):
                _finger_paths.append(_ps)
        _bound = self._bind_physics_material(
            '/GripperFinger', _finger_paths)
        print(
            '[grip-fric] bound finger material to %d prims: %s' % (len(_bound), _bound), flush=True
        )

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
                print('[grip-col] convexHull collider (薄いまま) %s' %
                      _cp, flush=True)
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
        # 旧設定 (k=8, d=28) は時定数 d/k=3.5s で閉じ切りに ~16 秒かかっていた。
        # 柔らかさ(コンプライアンス)は保ちつつ時定数を 0.5s に短縮し、
        # 開閉を実用速度 (~2 秒) にする。物体を弾くようなら k を下げる。
        FINGER_STIFFNESS = 20.0
        FINGER_DAMPING = 10.0
        FINGER_MAX_FORCE = 10.0
        finger_joint_paths = [
            self.stage_path + self.prefix + '/hand_palm_link/hand_l_proximal_joint',
            self.stage_path + self.prefix + '/hand_l_mimic_distal_link/hand_l_distal_joint',
            self.stage_path + self.prefix + '/hand_palm_link/hand_r_proximal_joint',
            self.stage_path + self.prefix + '/hand_r_mimic_distal_link/hand_r_distal_joint',
        ]
        for _fp in finger_joint_paths:
            _fd = UsdPhysics.DriveAPI.Get(
                stage.get_current_stage().GetPrimAtPath(_fp), 'angular')
            if not _fd:
                print('[grip-drive] DriveAPI not found: %s' % _fp, flush=True)
                continue
            _fd.GetStiffnessAttr().Set(FINGER_STIFFNESS)
            _fd.GetDampingAttr().Set(FINGER_DAMPING)
            _fd.GetMaxForceAttr().Set(FINGER_MAX_FORCE)
            print(
                '[grip-drive] tuned %s (k=%.1f d=%.1f maxF=%.1f)'
                % (_fp, FINGER_STIFFNESS, FINGER_DAMPING, FINGER_MAX_FORCE),
                flush=True,
            )

        left_passive1_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path
                + self.prefix
                + '/base_l_passive_wheel_x_frame/base_l_passive_wheel_y_frame_joint'
            ),
            'angular',
        )
        # 転がり軸: damping=0 だと止まらなくなるので小さな粘性摩擦を入れる
        # (詳細は _BASE_PASSIVE_WHEEL_DAMPING の定義箇所のコメント)。
        left_passive1_drive.GetDampingAttr().Set(_BASE_PASSIVE_WHEEL_DAMPING)
        left_passive1_drive.GetStiffnessAttr().Set(0)

        left_passive2_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path
                + self.prefix
                + '/base_l_passive_wheel_y_frame/base_l_passive_wheel_z_joint'
            ),
            'angular',
        )
        # 旋回軸: 実測で正しく減衰して止まるので無摩擦のままにする
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
        # 転がり軸: damping=0 だと止まらなくなるので小さな粘性摩擦を入れる
        # (詳細は _BASE_PASSIVE_WHEEL_DAMPING の定義箇所のコメント)。
        right_passive1_drive.GetDampingAttr().Set(_BASE_PASSIVE_WHEEL_DAMPING)
        right_passive1_drive.GetStiffnessAttr().Set(0)

        right_passive2_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path
                + self.prefix
                + '/base_r_passive_wheel_y_frame/base_r_passive_wheel_z_joint'
            ),
            'angular',
        )
        # 旋回軸: 実測で正しく減衰して止まるので無摩擦のままにする
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

        # Imported defaults make lateral steering effectively bang-bang.
        # Respect the URDF effort limits so velocity error cannot kick the
        # chassis through the high-friction drive wheels.
        wheel_damping = (
            _BASE_DIRECT_JOINT_DAMPING
            if _BASE_DIRECT_DRIVE
            else _BASE_WHEEL_DRIVE_DAMPING
        )
        steer_damping = (
            _BASE_DIRECT_JOINT_DAMPING
            if _BASE_DIRECT_DRIVE
            else _BASE_STEER_DRIVE_DAMPING
        )
        left_wheel_drive.GetDampingAttr().Set(wheel_damping)
        right_wheel_drive.GetDampingAttr().Set(wheel_damping)
        roll_drive.GetDampingAttr().Set(steer_damping)
        wheel_max_force = (
            _BASE_DIRECT_JOINT_MAX_FORCE
            if _BASE_DIRECT_DRIVE
            else _BASE_WHEEL_DRIVE_MAX_FORCE
        )
        steer_max_force = (
            _BASE_DIRECT_JOINT_MAX_FORCE
            if _BASE_DIRECT_DRIVE
            else _BASE_STEER_DRIVE_MAX_FORCE
        )
        left_wheel_drive.GetMaxForceAttr().Set(wheel_max_force)
        right_wheel_drive.GetMaxForceAttr().Set(wheel_max_force)
        roll_drive.GetMaxForceAttr().Set(steer_max_force)

        left_wheel_drive.GetStiffnessAttr().Set(0)
        right_wheel_drive.GetStiffnessAttr().Set(0)
        roll_drive.GetStiffnessAttr().Set(0)
        print(
            '[base-drive] damping=(wheel:%.3g, steer:%.3g) '
            'maxForce=(wheel:%.3g, steer:%.3g)'
            % (
                wheel_damping,
                steer_damping,
                wheel_max_force,
                steer_max_force,
            ),
            flush=True,
        )
