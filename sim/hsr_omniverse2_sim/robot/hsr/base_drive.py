"""台車駆動 (kinematic anchor / ブレーキ / 壁クランプ / リセット) (hsr.py から分離した BaseDriveMixin)。

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
from hsr_omniverse2_sim.robot.hsr.params import (_BASE_ANGULAR_ACCEL_LIMIT, _BASE_BRAKE_ANGULAR_C, _BASE_BRAKE_ANGULAR_K, _BASE_BRAKE_C, _BASE_BRAKE_CREEP_ANGULAR, _BASE_BRAKE_CREEP_SPEED, _BASE_BRAKE_ENABLED, _BASE_BRAKE_ENGAGE_ANGULAR, _BASE_BRAKE_ENGAGE_LINEAR, _BASE_BRAKE_K, _BASE_CMD_TAU, _BASE_COLLISION_HEIGHT, _BASE_COLLISION_MARGIN, _BASE_COLLISION_RADIUS, _BASE_CONTROL_DEADBAND, _BASE_DIAG, _BASE_DIRECT_DRIVE, _BASE_DIRECT_ROOT_DAMPING, _BASE_KINEMATIC_ANCHOR_MODE, _BASE_KINEMATIC_COLLISION, _BASE_KINEMATIC_DRIVE, _BASE_LINEAR_ACCEL_LIMIT, _BASE_RESET_HOLD_TIME, _BASE_SETPOINT_MAX_LAG, _BASE_SETPOINT_MAX_LAG_ANGULAR, _BASE_TRAJ_D_GAIN, _BASE_TRAJ_I_ANGULAR_LIMIT, _BASE_TRAJ_I_GAIN, _BASE_TRAJ_I_LINEAR_LIMIT, _BASE_TRAJ_P_GAIN, _GT_ODOM)
from hsr_omniverse2_sim.robot.hsr.quat import euler_from_quaternion
from hsr_omniverse2_sim.robot.hsr.kinematics import CartSpace, JointSpace


class BaseDriveMixin:
    def _reset_base_pose_integral(self):
        self._base_pose_error_integral = [0.0, 0.0, 0.0]
    def _dump_physics_report(self):
        """台車挙動の解析用に、シミュレータ側の物理パラメータを一括で出力する。

        質量・慣性・ソルバー設定・関節ドライブ・摩擦が分かると、台車が腕の反力で
        どれだけ動くかが数値で説明できる (BASE_PHYSICS_REPORT=0 で抑制)。
        """
        def _p(line):
            print('[phys] ' + line, flush=True)

        _p('================ physics report ================')
        try:
            _p('physics_dt=%.6f s (%.1f Hz)  rendering_dt=%.6f s'
               % (self.simulation_context.get_physics_dt(),
                  1.0 / max(1e-9, self.simulation_context.get_physics_dt()),
                  self.simulation_context.get_rendering_dt()))
        except Exception as exc:
            _p('timestep 取得失敗: %s' % exc)

        try:
            scene_prim = None
            for prim in stage.get_current_stage().Traverse():
                if prim.IsA(UsdPhysics.Scene):
                    scene_prim = prim
                    break
            if scene_prim is not None:
                usd_scene = UsdPhysics.Scene(scene_prim)
                _p('scene=%s gravity_dir=%s magnitude=%s'
                   % (scene_prim.GetPath(),
                      usd_scene.GetGravityDirectionAttr().Get(),
                      usd_scene.GetGravityMagnitudeAttr().Get()))
                px_scene = PhysxSchema.PhysxSceneAPI(scene_prim)
                for attr in (
                    'GetTimeStepsPerSecondAttr',
                    'GetSolverTypeAttr',
                    'GetEnableGPUDynamicsAttr',
                    'GetEnableStabilizationAttr',
                    'GetBounceThresholdAttr',
                    'GetFrictionOffsetThresholdAttr',
                    'GetMinPositionIterationCountAttr',
                    'GetMaxPositionIterationCountAttr',
                    'GetMinVelocityIterationCountAttr',
                    'GetMaxVelocityIterationCountAttr',
                ):
                    try:
                        a = getattr(px_scene, attr)()
                        _p('  scene.%s = %s' % (a.GetName(), a.Get()))
                    except Exception:
                        pass
        except Exception as exc:
            _p('scene 取得失敗: %s' % exc)

        # --- 質量・慣性 (台車が反力でどれだけ動くかを決める最重要値) ---
        try:
            names = list(self.robots.body_names)
            masses = self.robots.get_body_masses()[0]
            total = float(sum(float(m) for m in masses))
            _p('articulation=%s links=%d total_mass=%.3f kg'
               % (self.robots.prim_paths[0], len(names), total))
            inertias = None
            try:
                inertias = self.robots.get_body_inertias()[0]
            except Exception:
                pass
            for i, name in enumerate(names):
                mass = float(masses[i])
                if mass <= 0.0:
                    continue
                extra = ''
                if inertias is not None:
                    iv = [float(v) for v in inertias[i]]
                    if len(iv) == 9:
                        extra = ' I=(%.4f, %.4f, %.4f)' % (iv[0], iv[4], iv[8])
                _p('  link %-28s m=%8.3f kg%s' % (name, mass, extra))
        except Exception as exc:
            _p('質量取得失敗: %s' % exc)

        # --- 関節ドライブ (腕の駆動力 = 台車を押す力の源) ---
        try:
            for name, (ptr, _jt, _inv) in sorted(self._joints.items()):
                props = self.dc.get_dof_properties(ptr)
                _p('  dof %-28s k=%-10.6g d=%-10.6g maxEffort=%-10.6g '
                   'maxVel=%-8.6g limits=[%.3f, %.3f]'
                   % (name, props.stiffness, props.damping,
                      props.max_effort, props.max_velocity,
                      props.lower, props.upper))
        except Exception as exc:
            _p('dof 取得失敗: %s' % exc)

        # --- 台車設定値 (環境変数で調整しているもの) ---
        _p('base: direct_drive=%s kinematic_drive=%s brake=%s gt_odom=%s'
           % (_BASE_DIRECT_DRIVE, _BASE_KINEMATIC_DRIVE,
              _BASE_BRAKE_ENABLED, _GT_ODOM))
        _p('base: traj P=%.3f I=%.3f D=%.3f deadband=%.4f'
           % (_BASE_TRAJ_P_GAIN, _BASE_TRAJ_I_GAIN, _BASE_TRAJ_D_GAIN,
              _BASE_CONTROL_DEADBAND))
        _p('base: cmd_tau=%.3f accel_lin=%.3f accel_ang=%.3f root_damping=%.3f'
           % (_BASE_CMD_TAU, _BASE_LINEAR_ACCEL_LIMIT,
              _BASE_ANGULAR_ACCEL_LIMIT, _BASE_DIRECT_ROOT_DAMPING))
        _p('base: joint_drive k=%.0f d=%.0f (ang k=%.0f d=%.0f) '
           'setpoint_lag=%.3fm'
           % (_BASE_BRAKE_K, _BASE_BRAKE_C, _BASE_BRAKE_ANGULAR_K,
              _BASE_BRAKE_ANGULAR_C, _BASE_SETPOINT_MAX_LAG))
        _p('base: brake_engage=(%.3f m, %.3f rad) creep=(%.3f m/s, %.3f rad/s)'
           % (_BASE_BRAKE_ENGAGE_LINEAR, _BASE_BRAKE_ENGAGE_ANGULAR,
              _BASE_BRAKE_CREEP_SPEED, _BASE_BRAKE_CREEP_ANGULAR))
        if _BASE_KINEMATIC_DRIVE:
            _p('base: kinematic_collision=%s radius=%.3fm height=%.3fm '
               'margin=%.3fm'
               % (_BASE_KINEMATIC_COLLISION, _BASE_COLLISION_RADIUS,
                  _BASE_COLLISION_HEIGHT, _BASE_COLLISION_MARGIN))
            _p('base: kinematic_anchor_mode=%s'
               % (_BASE_KINEMATIC_ANCHOR_MODE,))
        _p('===============================================')
    def _apply_base_brake(self, target, dt):
        """駐車ブレーキ: 停止すべき間、台車を保持位置へバネで固定する。

        直接駆動モードはタイヤ摩擦が 0 なので、速度をゼロに書くだけでは 1 物理
        ステップ内に生じる腕の反力を受け止められず毎ステップ数 mm ずつ流される
        (実測: 腕を振ると最大 1055mm)。平面ジョイントのドライブで保持すると
        拘束ソルバーの中で反力を受け止められ、3.9mm に収まる。移動指令中は
        解除するので衝突応答は従来どおり。
        """
        pin = getattr(self, '_base_brake_pin', None)
        if pin is None:
            # 掛け始めは現在位置から。いきなり目標へ飛ばすと段差になるため。
            pin = (
                self.odometry_estimator.pose.x,
                self.odometry_estimator.pose.y,
                self.odometry_estimator.pose.ang,
            )
            if _BASE_DIAG:
                print(
                    '[base-brake] 保持開始 (%.4f, %.4f, %.4f)' % pin,
                    flush=True,
                )
        # 残った誤差はゆっくり詰める (瞬間移動させない)。
        if dt > 0.0 and _BASE_BRAKE_CREEP_SPEED > 0.0:
            ex = target[0] - pin[0]
            ey = target[1] - pin[1]
            er = math.atan2(
                math.sin(target[2] - pin[2]), math.cos(target[2] - pin[2]))
            max_lin = _BASE_BRAKE_CREEP_SPEED * dt
            dist = math.hypot(ex, ey)
            if dist > max_lin:
                scale = max_lin / dist
                ex *= scale
                ey *= scale
            max_ang = _BASE_BRAKE_CREEP_ANGULAR * dt
            er = max(-max_ang, min(max_ang, er))
            pin = (pin[0] + ex, pin[1] + ey, pin[2] + er)
        self._base_brake_pin = pin
        self._base_brake_engaged = True

        # 保持の実体はこれ。平面ジョイントのドライブ(バネ・ダンパ)を保持位置へ
        # 向けると、腕の反力を拘束ソルバーの中で受け止められる。
        # (Dynamic Control の外力印加も試したが、2699N 掛けても台車が止まらず
        #  効果ゼロだったため使わない。速度の書き戻しも隣接リンクの慣性に
        #  打ち消される。実際に効くのはこのドライブだけ。)
        self._set_joint_brake_target(pin)

        # 速度ゼロの書き戻しは単独では効かないが、ドライブの補助にはなる。
        try:
            self.robots.set_velocities(np.zeros((1, 6), dtype=np.float32))
        except Exception as exc:
            self._warn_once('brake-vel', 'ブレーキ速度ゼロ化に失敗: %s' % exc)

        if _BASE_DIAG:
            self._base_brake_diag_count = (
                getattr(self, '_base_brake_diag_count', 0) + 1)
            if self._base_brake_diag_count % 30 == 1:
                pose = self.odometry_estimator.pose
                print(
                    '[base-diag] 保持位置=(%.4f, %.4f, %.4f) 実位置=(%.4f,'
                    ' %.4f, %.4f) ずれ=(%.4f, %.4f, %.4f)'
                    % (pin[0], pin[1], pin[2], pose.x, pose.y, pose.ang,
                       pin[0] - pose.x, pin[1] - pose.y,
                       math.atan2(math.sin(pin[2] - pose.ang),
                                  math.cos(pin[2] - pose.ang))),
                    flush=True,
                )
    def _advance_base_setpoint(self, cmd, dt):
        """指令速度を積分して台車の目標位置 (setpoint) を進める。

        実位置から離れすぎないよう制限する (アンチワインドアップ)。壁に当たって
        止まったときに目標だけが先へ進み、解放された瞬間に大きな力で飛び出す
        のを防ぐ。摩擦 0 の台車なので、この程度の差でも加速力は十分足りる。
        """
        pose = self.odometry_estimator.pose
        setpoint = getattr(self, '_base_setpoint', None)
        if setpoint is None:
            setpoint = (pose.x, pose.y, pose.ang)
        if dt > 0.0:
            setpoint = (
                setpoint[0] + cmd.dot_x * dt,
                setpoint[1] + cmd.dot_y * dt,
                setpoint[2] + cmd.dot_r * dt,
            )
        lag_x = setpoint[0] - pose.x
        lag_y = setpoint[1] - pose.y
        lag_r = math.atan2(
            math.sin(setpoint[2] - pose.ang),
            math.cos(setpoint[2] - pose.ang))
        lag = math.hypot(lag_x, lag_y)
        if _BASE_SETPOINT_MAX_LAG > 0.0 and lag > _BASE_SETPOINT_MAX_LAG:
            scale = _BASE_SETPOINT_MAX_LAG / lag
            lag_x *= scale
            lag_y *= scale
        lag_r = max(-_BASE_SETPOINT_MAX_LAG_ANGULAR,
                    min(_BASE_SETPOINT_MAX_LAG_ANGULAR, lag_r))
        self._base_setpoint = (
            pose.x + lag_x, pose.y + lag_y, pose.ang + lag_r)
    def _kinematic_odom_to_world(self, target):
        """odom 系の平面姿勢 (x, y, yaw) を anchor の world 姿勢へ変換する。

        GT odom の基準変換を逆にたどる。基準取得前だけは現在の base_footprint
        から差分で求めるので、そのフレームだけ dc への問い合わせが要る。
        """
        ref = getattr(self, '_gt_ref', None)
        if ref is not None:
            rwx, rwy, rwyaw, rox, roy, royaw = ref
            odx = target[0] - rox
            ody = target[1] - roy
            coso = math.cos(royaw)
            sino = math.sin(royaw)
            relx = odx * coso + ody * sino
            rely = -odx * sino + ody * coso
            cosw = math.cos(rwyaw)
            sinw = math.sin(rwyaw)
            world_x = rwx + relx * cosw - rely * sinw
            world_y = rwy + relx * sinw + rely * cosw
            world_yaw = rwyaw + math.atan2(
                math.sin(target[2] - royaw),
                math.cos(target[2] - royaw),
            )
            return world_x, world_y, world_yaw

        pose = self.odometry_estimator.pose
        body = getattr(self, '_gt_body', None)
        if not body:
            body = self.dc.get_rigid_body(
                self.stage_path + self.prefix + '/base_footprint')
        current = self.dc.get_rigid_body_pose(body)
        current_yaw = euler_from_quaternion(
            current.r.x, current.r.y, current.r.z, current.r.w)[2]
        frame_yaw = current_yaw - pose.ang
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        world_x = current.p.x + (
            dx * math.cos(frame_yaw) - dy * math.sin(frame_yaw))
        world_y = current.p.y + (
            dx * math.sin(frame_yaw) + dy * math.cos(frame_yaw))
        world_yaw = current_yaw + math.atan2(
            math.sin(target[2] - pose.ang),
            math.cos(target[2] - pose.ang),
        )
        return world_x, world_y, world_yaw
    def _anchor_xform_api(self):
        """anchor prim の XformCommonAPI (生成時と同じ translate + rotateXYZ)。"""
        api = getattr(self, '_base_anchor_xform_api', None)
        if api is not None:
            return api
        prim = stage.get_current_stage().GetPrimAtPath(
            self._base_kinematic_anchor_path)
        if not prim or not prim.IsValid():
            raise RuntimeError(
                'kinematic anchor prim is unavailable: %s'
                % self._base_kinematic_anchor_path)
        api = UsdGeom.XformCommonAPI(prim)
        self._base_anchor_xform_api = api
        return api
    def _verify_anchor_tracking(self, anchor, world_x, world_y):
        """usd モードで anchor が実際に追従しているかを起動直後だけ確かめる。

        USD→PhysX の同期が効かない環境では台車が全く動かなくなるので、指令した
        変位に対して実変位が出ていないことが続いたら dc モードへ戻す。
        """
        if getattr(self, '_base_anchor_mode_verified', False):
            return
        previous_cmd = getattr(self, '_base_anchor_cmd_prev', None)
        previous_actual = getattr(self, '_base_anchor_actual_prev', None)
        self._base_anchor_cmd_prev = (world_x, world_y)
        try:
            pose = self.dc.get_rigid_body_pose(anchor)
            actual = (float(pose.p.x), float(pose.p.y))
        except Exception:
            return
        self._base_anchor_actual_prev = actual
        if previous_cmd is None or previous_actual is None:
            return
        commanded = math.hypot(
            world_x - previous_cmd[0], world_y - previous_cmd[1])
        if commanded < 1e-4:
            return
        moved = math.hypot(
            actual[0] - previous_actual[0], actual[1] - previous_actual[1])
        if moved >= 0.1 * commanded:
            self._base_anchor_mode_verified = True
            return
        fails = getattr(self, '_base_anchor_track_fails', 0) + 1
        self._base_anchor_track_fails = fails
        if fails >= 30:
            self._base_anchor_mode = 'dc'
            self._base_anchor_mode_verified = True
            self._warn_once(
                'kinematic-anchor-mode',
                'USD 経由の kinematic target が反映されないため、'
                'anchor の駆動を dc (瞬間移動) へ戻しました',
            )
    def _set_kinematic_base_pose(self, target):
        """odom の目標姿勢へ kinematic anchor を移す。

        anchor は kinematic body なので速度は書けない (PhysX の
        PxRigidDynamic::setLinearVelocity は "Body must be non-kinematic!" で
        弾き、1000 件でシミュレーション自体が停止する)。代わりに usd モードでは
        USD の xform を書いて PhysX に kinematic target として解釈させ、PhysX 自身
        に速度を導出させる。
        """
        if not all(math.isfinite(float(value)) for value in target):
            self._warn_once(
                'kinematic-nonfinite-target',
                '非有限の台車目標を破棄しました: %r' % (target,),
            )
            return False
        try:
            anchor = getattr(self, '_base_kinematic_anchor', None)
            if not anchor:
                anchor = self.dc.get_rigid_body(
                    self._base_kinematic_anchor_path)
                if not anchor:
                    raise RuntimeError('kinematic anchor handle is unavailable')
                self._base_kinematic_anchor = anchor

            world_x, world_y, world_yaw = self._kinematic_odom_to_world(target)
            # _gt_ref が NaN で汚染されると target が有限でもここが NaN になる。
            # そのまま PhysX へ書くと Invalid PhysX transform でシーンごと発散
            # するので、書く直前にもう一度検査する。
            if not all(math.isfinite(float(value))
                       for value in (world_x, world_y, world_yaw)):
                self._warn_once(
                    'kinematic-nonfinite-world',
                    '非有限の台車 world 姿勢を破棄しました: %r'
                    % ((world_x, world_y, world_yaw),),
                )
                return False

            mode = getattr(self, '_base_anchor_mode', None)
            if mode is None:
                mode = _BASE_KINEMATIC_ANCHOR_MODE
                self._base_anchor_mode = mode
            if mode == 'usd':
                api = self._anchor_xform_api()
                api.SetTranslate(
                    Gf.Vec3d(float(world_x), float(world_y), 0.0))
                api.SetRotate(
                    Gf.Vec3f(0.0, 0.0, math.degrees(float(world_yaw))),
                    UsdGeom.XformCommonAPI.RotationOrderXYZ,
                )
                self._verify_anchor_tracking(anchor, world_x, world_y)
            else:
                transform = _dynamic_control.Transform()
                transform.p = (float(world_x), float(world_y), 0.0)
                half = 0.5 * world_yaw
                transform.r = (0.0, 0.0, math.sin(half), math.cos(half))
                self.dc.set_rigid_body_pose(anchor, transform)
        except Exception as exc:
            self._warn_once(
                'kinematic-drive',
                '決定論的な台車姿勢更新に失敗; 従来制御へ戻します: %s' % exc,
            )
            return False
        return True
    def _physx_scene_query(self):
        """PhysX のシーンクエリインタフェース。使えなければ None。"""
        if hasattr(self, '_physx_query_iface'):
            return self._physx_query_iface
        query = None
        try:
            from omni.physx import get_physx_scene_query_interface
            query = get_physx_scene_query_interface()
        except Exception as exc:
            self._warn_once(
                'kinematic-collision-iface',
                'PhysX シーンクエリを取得できず、台車の衝突クランプを無効化'
                'します: %s' % exc,
            )
        self._physx_query_iface = query
        return query
    def _kinematic_sweep_scale(self, start_world, end_world):
        """start→end の並進を障害物の手前で止める比 (0.0-1.0) を返す。

        anchor は衝突形状を持たず、台車は FixedJoint で無限質量に溶接されている
        ため、PhysX の接触拘束では止まらない。代わりに台車の外周を覆う球を
        スイープし、貫通する分だけ目標の前進量を削る。回転は球スイープでは
        検出できないので通す。
        """
        dx = float(end_world[0]) - float(start_world[0])
        dy = float(end_world[1]) - float(start_world[1])
        distance = math.hypot(dx, dy)
        if distance <= 1e-9 or _BASE_COLLISION_RADIUS <= 0.0:
            return 1.0
        query = self._physx_scene_query()
        if query is None:
            return 1.0

        origin = (float(start_world[0]), float(start_world[1]),
                  _BASE_COLLISION_HEIGHT)
        direction = (dx / distance, dy / distance, 0.0)
        # None のまま帰ってきたら「進路上に何も無い」= クランプしない。
        limit = [None]

        def _report(hit):
            # SceneQueryHitObject の属性は rigid_body / collision (snake_case)。
            # 壁は静的コライダーなので rigid_body は空で、collision だけが埋まる。
            path = str(getattr(hit, 'rigid_body', '')
                       or getattr(hit, 'collision', ''))
            # 自分自身のコライダーは無視する。初期接触 (distance≈0) も無視:
            # 壁際で止まっているときに脱出方向まで塞いでしまうため。余裕
            # (_BASE_COLLISION_MARGIN) を残して停めるので、押し込みは次の
            # ステップのスイープが正の距離で捕まえる。
            if path == '/hsrb' or path.startswith('/hsrb/'):
                return True
            hit_distance = float(getattr(hit, 'distance', 0.0))
            if hit_distance <= 1e-4:
                return True
            if limit[0] is None or hit_distance < limit[0]:
                limit[0] = hit_distance
            return True

        if not self._sweep_sphere(
                query, _BASE_COLLISION_RADIUS, origin, direction, distance,
                _report):
            return 1.0
        if limit[0] is None:
            return 1.0

        allowed = min(distance, max(0.0, limit[0] - _BASE_COLLISION_MARGIN))
        return allowed / distance
    def _sweep_sphere(self, query, radius, origin, direction, distance,
                      report):
        """全ヒットを report へ渡す球スイープ。成功したら True。

        Isaac Sim 4.5 の PhysXSceneQuery は
        sweep_sphere_all(radius, origin, dir, distance, reportFn, bothSides)
        で、ベクトルは carb.Float3。旧名 sweep_sphere も一応試す。
        """
        fn = getattr(self, '_sweep_sphere_fn', None)
        if fn is None:
            for name in ('sweep_sphere_all', 'sweep_sphere'):
                fn = getattr(query, name, None)
                if fn is not None:
                    break
            if fn is None:
                self._warn_once(
                    'kinematic-collision-sweep',
                    '球スイープ API が見つからず、台車の衝突クランプを無効化'
                    'します (利用可能: %s)'
                    % ([n for n in dir(query) if 'sweep' in n],),
                )
                self._physx_query_iface = None
                return False
            self._sweep_sphere_fn = fn

        try:
            import carb
            fn(
                float(radius),
                carb.Float3(*origin),
                carb.Float3(*direction),
                float(distance),
                report,
                False,
            )
        except Exception as exc:
            self._warn_once(
                'kinematic-collision-sweep',
                '台車の衝突スイープに失敗し、クランプを無効化します: %s' % exc,
            )
            # 毎ステップ例外を投げ続けないよう、以後はクランプを止める。
            self._physx_query_iface = None
            self._sweep_sphere_fn = None
            return False
        return True
    def _apply_kinematic_base_motion(self, cmd, dt):
        """平滑化済み速度を積分し、kinematic anchor の平面姿勢を更新する。

        速度書き込みと平面ジョイントの位置ドライブを併用すると拘束ソルバー内で
        競合し、純回転だけでも数十 cm 並進する。移動中は衝突しない anchor の
        目標姿勢だけを進めることで、その競合をなくす。
        """
        if dt <= 0.0:
            return False
        pose = self.odometry_estimator.pose
        motion_values = (
            cmd.dot_x, cmd.dot_y, cmd.dot_r,
            pose.x, pose.y, pose.ang,
        )
        if not all(math.isfinite(float(value)) for value in motion_values):
            # Returning True is intentional: the kinematic path handled the
            # bad command by stopping, so the legacy velocity writer must not
            # receive the same NaN as a fallback.
            self._warn_once(
                'kinematic-nonfinite-command',
                '非有限の台車指令を停止へ置換しました: %r' %
                (motion_values,),
            )
            self._base_direct_pose = None
            self._base_kinematic_velocity = (0.0, 0.0, 0.0)
            return True
        previous = getattr(self, '_base_direct_pose', None)
        if previous is None:
            previous = (pose.x, pose.y, pose.ang)
        candidate = (
            previous[0] + cmd.dot_x * dt,
            previous[1] + cmd.dot_y * dt,
            math.atan2(
                math.sin(previous[2] + cmd.dot_r * dt),
                math.cos(previous[2] + cmd.dot_r * dt),
            ),
        )

        # world 系での前進を求め、障害物を貫通する分だけ削る。world 変換には
        # _gt_ref が要るので、基準取得前 (起動直後の 1 フレーム) は素通しする。
        scale = 1.0
        if _BASE_KINEMATIC_COLLISION and getattr(self, '_gt_ref', None):
            try:
                scale = self._kinematic_sweep_scale(
                    self._kinematic_odom_to_world(previous),
                    self._kinematic_odom_to_world(candidate),
                )
            except Exception as exc:
                self._warn_once(
                    'kinematic-collision',
                    '台車の衝突クランプに失敗し、素通しします: %s' % exc,
                )
                scale = 1.0

        if scale < 1.0:
            target = (
                previous[0] + (candidate[0] - previous[0]) * scale,
                previous[1] + (candidate[1] - previous[1]) * scale,
                candidate[2],
            )
        else:
            target = candidate

        if not self._set_kinematic_base_pose(target):
            self._base_direct_pose = None
            self._base_kinematic_velocity = (0.0, 0.0, 0.0)
            return False

        # クランプ後の姿勢を目標として持ち越す。これが kinematic 経路の
        # アンチワインドアップで、壁の向こうへ目標が積分され続けるのと、
        # 解放された瞬間に飛び出すのを防ぐ。
        self._base_direct_pose = target
        self._base_setpoint = target
        # odom の twist はここから出る (下流で /odom へ流れる)。指令値ではなく
        # 実現できた速度を返し、壁で止まったことをナビゲーション側から
        # 観測できるようにする。
        self._base_kinematic_velocity = (
            cmd.dot_x * scale, cmd.dot_y * scale, cmd.dot_r)
        # クランプの出入りだけを出す (60Hz で毎ステップ出すとログが埋まる)。
        clamped = scale < 1.0
        if _BASE_DIAG and clamped != getattr(self, '_base_clamp_prev', False):
            print(
                '[base-diag] kinematic collision clamp %s scale=%.3f '
                'cmd=(%.3f, %.3f, %.3f)'
                % ('ON' if clamped else 'OFF', scale,
                   cmd.dot_x, cmd.dot_y, cmd.dot_r),
                flush=True,
            )
        self._base_clamp_prev = clamped
        # 平面ジョイント側も同じ姿勢へ合わせ、次の物理ステップで姿勢更新を
        # 打ち消す力が出ないようにする。
        self._set_joint_brake_target(target)
        return True
    def _odom_to_joint(self, pose):
        """odom 姿勢 → 平面ジョイントの DOF 値。

        ジョイントの原点は spawn 姿勢 (frame0)、odom の原点は起動直後に
        真値odomを初めて読んだ時点の姿勢で、両者は数 cm ずれる。この差を
        足さないとサーボの指令が常にずれ、台車が目標に到達できなくなる
        (実測: 6.6cm ずれた状態で reset 後の動作が全て失敗した)。
        """
        ref = getattr(self, '_gt_ref', None)
        frame0 = getattr(self, '_planar_frame0', None)
        if ref is None or frame0 is None:
            return pose
        # ref = (world_x, world_y, world_yaw, odom_x, odom_y, odom_yaw)
        # odom 原点に対応する world 姿勢と frame0 の差が、そのままオフセット。
        return (
            pose[0] + (ref[0] - ref[3]) - frame0[0],
            pose[1] + (ref[1] - ref[4]) - frame0[1],
            pose[2] + (ref[2] - ref[5]) - frame0[2],
        )
    def _set_joint_brake_target(self, pose, velocity=(0.0, 0.0, 0.0)):
        """平面ジョイントのドライブ目標 (位置・速度) を設定する。

        引数は odom 姿勢。内部でジョイントの DOF 値へ直してから渡す。
        出力 = 剛性 x (目標位置 - 現在位置) + 減衰 x (目標速度 - 現在速度)。
        停止中は目標速度 0 で現在位置に固定 = 駐車ブレーキ。走行中は指令速度を
        目標速度に入れておけば減衰項が走行を妨げない。
        """
        drives = getattr(self, '_base_brake_drives', None)
        if not drives:
            return
        pose = self._odom_to_joint(pose)
        state = (tuple(pose), tuple(velocity))
        if getattr(self, '_base_brake_drive_state', None) == state:
            return
        try:
            drives['transX'].GetTargetPositionAttr().Set(float(pose[0]))
            drives['transY'].GetTargetPositionAttr().Set(float(pose[1]))
            drives['rotZ'].GetTargetPositionAttr().Set(
                math.degrees(math.atan2(
                    math.sin(pose[2]), math.cos(pose[2]))))
            drives['transX'].GetTargetVelocityAttr().Set(float(velocity[0]))
            drives['transY'].GetTargetVelocityAttr().Set(float(velocity[1]))
            drives['rotZ'].GetTargetVelocityAttr().Set(
                math.degrees(velocity[2]))
            self._base_brake_drive_state = state
        except Exception as exc:
            self._warn_once(
                'brake-drive', 'ブレーキドライブ目標の設定に失敗: %s' % exc)
    def _warn_once(self, key, message):
        seen = getattr(self, '_warned_keys', None)
        if seen is None:
            seen = set()
            self._warned_keys = seen
        if key in seen:
            return
        seen.add(key)
        print('[base-brake] ' + message, flush=True)
    def _release_base_brake(self, reason=''):
        if not getattr(self, '_base_brake_engaged', False):
            self._base_brake_pin = None
            return
        self._base_brake_engaged = False
        self._base_brake_pin = None
        # 走行再開時に古い目標へ引かれないよう、設定値を実位置に戻す。
        self._base_setpoint = (
            self.odometry_estimator.pose.x,
            self.odometry_estimator.pose.y,
            self.odometry_estimator.pose.ang,
        )
        if _BASE_DIAG:
            print('[base-brake] 保持解除%s'
                  % ((' (%s)' % reason) if reason else ''), flush=True)
    def _base_integral_command(self, dx, dy, dyaw, dt):
        if _BASE_TRAJ_I_GAIN <= 0.0 or dt <= 0.0:
            return (0.0, 0.0, 0.0)

        integral = self._base_pose_error_integral
        errors = (dx, dy, dyaw)
        for index, error in enumerate(errors):
            # Callers pass zero inside the control deadband.  Keeping the old
            # integral there drove the base through an already-reached goal.
            # Also discard compensation after crossing the target so it cannot
            # sustain a slow endpoint oscillation.
            if error == 0.0 or integral[index] * error < 0.0:
                integral[index] = 0.0
            else:
                integral[index] += error * dt

        linear_limit = (
            _BASE_TRAJ_I_LINEAR_LIMIT / _BASE_TRAJ_I_GAIN)
        linear_norm = math.hypot(integral[0], integral[1])
        if linear_limit <= 0.0:
            integral[0] = 0.0
            integral[1] = 0.0
        elif linear_norm > linear_limit:
            scale = linear_limit / linear_norm
            integral[0] *= scale
            integral[1] *= scale

        angular_limit = (
            _BASE_TRAJ_I_ANGULAR_LIMIT / _BASE_TRAJ_I_GAIN)
        integral[2] = max(
            -angular_limit, min(angular_limit, integral[2]))
        return tuple(_BASE_TRAJ_I_GAIN * value for value in integral)
    def reset_to_spawn(self, x, y, yaw):
        """ロボットを spawn 姿勢 (x, y, yaw) + ホーム関節へ戻し,速度をゼロに"""
        if not getattr(self, 'art', None):
            return

        # --- base (articulation root) をテレポートして速度ゼロ ---
        # Dynamic Control の剛体テレポートはルートリンクだけを動かすため、
        # 隣接リンクとの拘束が大きく破れ、PhysX がそれを解消する際にロボットが
        # 数 m 弾き飛ばされる (実測)。関節全体を一括で移動できる Tensor API を
        # 優先し、失敗したときだけ従来の方法にフォールバックする。
        _half = 0.5 * float(yaw)
        moved = False
        try:
            self.robots.set_world_poses(
                np.array([[float(x), float(y), 0.0]], dtype=np.float32),
                # Isaac の四元数は (w, x, y, z) 順
                np.array([[math.cos(_half), 0.0, 0.0, math.sin(_half)]],
                         dtype=np.float32),
            )
            self.robots.set_velocities(np.zeros((1, 6), dtype=np.float32))
            moved = True
        except Exception as exc:
            self._warn_once('reset-pose', 'リセットの一括移動に失敗: %s' % exc)

        if _BASE_KINEMATIC_DRIVE:
            try:
                anchor = getattr(self, '_base_kinematic_anchor', None)
                if not anchor:
                    anchor = self.dc.get_rigid_body(
                        self._base_kinematic_anchor_path)
                    self._base_kinematic_anchor = anchor
                anchor_pose = _dynamic_control.Transform()
                anchor_pose.p = (float(x), float(y), 0.0)
                anchor_pose.r = (
                    0.0, 0.0, math.sin(_half), math.cos(_half))
                self.dc.set_rigid_body_pose(anchor, anchor_pose)
            except Exception as exc:
                self._warn_once(
                    'reset-anchor',
                    'kinematic anchor のリセットに失敗: %s' % exc,
                )

        root = self.dc.get_articulation_root_body(self.art)
        if not moved:
            t = _dynamic_control.Transform()
            t.p = (float(x), float(y), 0.0)
            # dc の Transform.r は (x, y, z, w)。yaw (Z 軸) のみの回転。
            t.r = (0.0, 0.0, math.sin(_half), math.cos(_half))
            self.dc.set_rigid_body_pose(root, t)
        self.dc.set_rigid_body_linear_velocity(root, (0.0, 0.0, 0.0))
        self.dc.set_rigid_body_angular_velocity(root, (0.0, 0.0, 0.0))

        # --- 全関節をホーム姿勢へ・速度ゼロ・目標も合わせる ---
        home = getattr(self, '_home_dof_pos', {})
        for _name, (_ptr, _jt, _inv) in self._joints.items():
            _pos = home.get(_name, 0.0)
            self.dc.set_dof_position(_ptr, _pos)
            self.dc.set_dof_velocity(_ptr, 0.0)
            self.dc.set_dof_position_target(_ptr, _pos)
            self.dc.set_dof_velocity_target(_ptr, 0.0)

        # Base drives keep fixed velocity-drive properties for the articulation
        # lifetime.  Changing DOF properties while PhysX is running invalidates
        # the dynamic-control articulation handle in Isaac Sim 4.5.
        self._base_cmd_prev = JointSpace()
        self._base_direct_cmd_prev = CartSpace()
        self._base_direct_applied_cmd = CartSpace()
        self._base_direct_pose = None
        self._base_kinematic_velocity = (0.0, 0.0, 0.0)
        self._base_hold_pose = None
        # テレポート後に旧位置へバネで引き戻されないよう必ず解除し、
        # 平面ジョイントのドライブ目標も spawn 姿勢 (= joint frame0 の原点) へ
        # 移す。これをしないと剛性 20万 N/m のバネが旧位置へ引き戻し続け、
        # reset_world が効かなくなる。
        # 実行中の軌道は破棄する。残しておくとリセット後にロボットが
        # 「リセット前の目標」へ戻ろうとして、せっかく戻した位置から動き出す。
        for _srv in (
            getattr(self, 'odom_trajectory_action_server', None),
            getattr(self, 'arm_trajectory_action_server', None),
            getattr(self, 'head_trajectory_action_server', None),
            getattr(self, 'gripper_trajectory_action_server', None),
        ):
            if _srv is None:
                continue
            _handle = getattr(_srv, '_action_goal_handle', None)
            _srv._action_goal = None
            _srv._pending_topic_msg = None
            if _handle is not None and _handle.is_active:
                try:
                    _handle.abort()
                except Exception:
                    pass
            _srv._action_goal_handle = None
        self.cmd_vel_msg = None
        self.last_cmd_vel_time = 0.0

        # テレポートで Dynamic Control のハンドルが無効になると、真値odomの
        # 取得が例外で落ち、古い値のまま積分し続けて座標が壊れる。壊れた座標を
        # 位置サーボの目標に使うと台車が明後日の方向へ引かれるので、ハンドルを
        # 取り直し、オドメトリも spawn 原点へ戻す。
        self._gt_body = None
        self.odometry_estimator.pose.x = 0.0
        self.odometry_estimator.pose.y = 0.0
        self.odometry_estimator.pose.ang = 0.0

        self._release_base_brake()
        self._base_setpoint = (0.0, 0.0, 0.0)
        self._set_joint_brake_target((0.0, 0.0, 0.0))
        # 全関節を一瞬でホーム姿勢へ戻すと、その位置の飛びを PhysX が解消する
        # ときに大きな力が出て台車が弾かれる。落ち着くまでの短い間だけ、誤差の
        # 大小によらず spawn 位置へ固定し続ける。
        self._base_hold_pose = (0.0, 0.0, 0.0)
        self._post_reset_hold_until = (
            self.simulation_context.current_time + _BASE_RESET_HOLD_TIME)
        self._base_pose_error_integral = [0.0, 0.0, 0.0]
        self._base_trajectory_was_active = False
        self._joint_state_previous_positions = {}
        self._joint_state_filtered_velocities = {}

        self.dc.wake_up_articulation(self.art)

        # --- オドメトリ・速度指令をリセット ---
        # odom is local to the spawn pose.  Using world-space spawn x/y here
        # adds the placement offset a second time when _gt_ref is recaptured.
        self.odometry_estimator.set_pose(0.0, 0.0, 0.0)
        self.cmd_vel_msg = None
        self._gt_ref = None   # 真値odomの基準を再取得(テレポート後に対応づけ直す)

        # --- 把持中の物体があれば離す (衝突を戻し、追従を解除) ---
        if getattr(self, '_grasp_obj', None) is not None:
            try:
                self._set_grasp_object_collision(self._grasp_obj['path'], True)
            except Exception:
                pass
            self._grasp_obj = None
