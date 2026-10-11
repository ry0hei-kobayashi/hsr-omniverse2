"""アタッチ把持と物体姿勢レポート (hsr.py から分離した GraspMixin)。

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
import tf_transformations
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
from hsr_math import _q_conj, _q_mul, _q_rot


class GraspMixin:
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
                if _ps == getattr(self, '_base_kinematic_anchor_path', None):
                    continue  # 衝突のない台車制御用anchorは把持対象ではない
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
                        _a = UsdPhysics.CollisionAPI(
                            _p).CreateCollisionEnabledAttr()
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
                self._joints['hand_motor_joint'][0], _dynamic_control.STATE_POS
            ).pos
        except Exception:
            return
        if self._palm_body is None:
            self._palm_body = self.dc.get_rigid_body(
                self.stage_path + self.prefix + '/hand_palm_link'
            )
        if getattr(self, '_lfinger_body', None) is None:
            self._lfinger_body = self.dc.get_rigid_body(
                self.stage_path + self.prefix + '/hand_l_distal_link'
            )
            self._rfinger_body = self.dc.get_rigid_body(
                self.stage_path + self.prefix + '/hand_r_distal_link'
            )
        if not self._palm_body or not self._lfinger_body or not self._rfinger_body:
            return
        _palm = self.dc.get_rigid_body_pose(self._palm_body)
        _pp = (_palm.p.x, _palm.p.y, _palm.p.z)
        _pq = (_palm.r.x, _palm.r.y, _palm.r.z, _palm.r.w)
        # 把持中心 = 左右の指先(distal)の中点。物体が来るべき場所そのもの。
        _lf = self.dc.get_rigid_body_pose(self._lfinger_body).p
        _rf = self.dc.get_rigid_body_pose(self._rfinger_body).p
        _gc = ((_lf.x + _rf.x) / 2.0, (_lf.y + _rf.y) /
               2.0, (_lf.z + _rf.z) / 2.0)

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
                    _d = math.sqrt(
                        (_gc[0] - _op.p.x) ** 2 + (_gc[1] -
                                                   _op.p.y) ** 2 + (_gc[2] - _op.p.z) ** 2
                    )
                    _mind = min(_mind, _d)
                    if _d < _bestd:
                        _bestd = _d
                        _best = (_bp, _h, _op)
                if _mind < 0.4 and not getattr(self, '_grasp_dbg_done', False):
                    print(
                        '[graspA] try: gc=(%.3f,%.3f,%.3f) nearest_obj_dist=%.3f (閾値%.2f)'
                        % (_gc[0], _gc[1], _gc[2], _mind, GRASP_DIST),
                        flush=True,
                    )
                    self._grasp_dbg_done = True
                if _best is not None:
                    _bp, _h, _op = _best
                    _rel_p = _q_rot(
                        _q_conj(_pq), (_op.p.x -
                                       _pp[0], _op.p.y - _pp[1], _op.p.z - _pp[2])
                    )
                    _rel_q = _q_mul(
                        _q_conj(_pq), (_op.r.x, _op.r.y, _op.r.z, _op.r.w))
                    self._grasp_obj = {
                        'h': _h, 'rel_p': _rel_p, 'rel_q': _rel_q, 'path': _bp}
                    # 把持中はテレポート保持なので衝突を切る(指との押し合いで発散しない)。
                    self._set_grasp_object_collision(_bp, False)
                    print('[graspA] 掴んだ: %s (dist=%.3f)' %
                          (_bp, _bestd), flush=True)
        else:
            if _hm > OPEN_T:  # 開いた → 離す
                try:
                    self.dc.set_rigid_body_linear_velocity(
                        self._grasp_obj['h'], (0.0, 0.0, 0.0))
                    self.dc.set_rigid_body_angular_velocity(
                        self._grasp_obj['h'], (0.0, 0.0, 0.0))
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
                self.dc.set_rigid_body_linear_velocity(
                    _g['h'], (0.0, 0.0, 0.0))
                self.dc.set_rigid_body_angular_velocity(
                    _g['h'], (0.0, 0.0, 0.0))
    def _dump_object_poses(self):
        """掴める物体の実際の位置 (world と odom) を一度だけ出力する。"""
        ref = getattr(self, '_gt_ref', None)
        print('[objects] 物体の実位置 (落下・転がり後):', flush=True)
        for path in self._object_body_paths():
            try:
                handle = self.dc.get_rigid_body(path)
                if not handle:
                    continue
                pose = self.dc.get_rigid_body_pose(handle)
            except Exception:
                continue
            name = path.strip('/').split('/')[0]
            line = ('[objects]   %-42s world=(%.3f, %.3f, %.3f)'
                    % (name, pose.p.x, pose.p.y, pose.p.z))
            if ref is not None:
                rwx, rwy, _rwyaw, rox, roy, _royaw = ref
                line += ('  odom=(%.3f, %.3f)'
                         % (rox + (pose.p.x - rwx), roy + (pose.p.y - rwy)))
            print(line, flush=True)
