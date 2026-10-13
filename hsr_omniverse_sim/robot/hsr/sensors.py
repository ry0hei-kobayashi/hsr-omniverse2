"""センサ (カメラ / IMU / LiDAR) の OmniGraph 構築 (hsr.py から分離した SensorsMixin)。

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
from hsr_omniverse_sim.robot.hsr.params import (_CAMERA_FRAME_SKIP, _ENABLE_STEREO_CAMERAS)


def og_ros_camera_info_node():
    """camera_info 用 OmniGraph ノードの型名。

    Isaac Sim 5.x では ROS2CameraHelper の ``inputs:type='camera_info'`` が廃止され
    (``OgnROS2CameraHelper: type is not supported``)、camera_info は専用ノード
    ROS2CameraInfoHelper で出す。rgb/depth 用の cameraHelper* と同じ renderProductPath
    を繋ぐだけでよく、``inputs:type`` は設定しない。
    """
    return 'isaacsim.ros2.bridge.ROS2CameraInfoHelper'


class SensorsMixin:
    def create_cameras(self) -> None:
        topic_prefix = ''
        print(
            '[hsr-camera] publish every %d rendered frame(s) '
            '(CAMERA_FRAME_SKIP=%d) stereo=%s'
            % (_CAMERA_FRAME_SKIP + 1, _CAMERA_FRAME_SKIP,
               'on' if _ENABLE_STEREO_CAMERAS else 'off'),
            flush=True,
        )

        # ステレオは既定 off (ENABLE_STEREO_CAMERAS=1 で on)。Camera prim を作らなければ
        # レンダープロダクトも OG グラフも作られないので、GPU のラスタライズも
        # ROS シリアライズもまるごと消える。
        if _ENABLE_STEREO_CAMERAS:
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
            # near=0.07m, far=100m。既定near=1mだと1m以内の近接物体が消えるので小さくする。
            # 頭部RGBDの自己オクルージョン対策と値を揃え、全カメラ0.07mに統一。far=100mは室内に十分。
            l_camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.07, 100.0))

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
            # near=0.07m, far=100m。既定near=1mだと1m以内の近接物体が消えるので小さくする。
            # 頭部RGBDの自己オクルージョン対策と値を揃え、全カメラ0.07mに統一。far=100mは室内に十分。
            r_camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.07, 100.0))

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
        # near=0.07m。nearを極小(1cm)にすると、頭部RGBDの広い視野では視界の上側に
        # ロボット自身の頭/体が映り込み「画像の上半分が黒い帯」になる(自己オクルージョン)。
        # 一方で既定の1mだと近接物体が消える。0.07mは近接物体を残しつつ自分の体を切る妥協点(実測)。
        # far=100mは室内に十分(深度精度も問題なし)。
        rgbd_camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.07, 100.0))

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
        # near=0.01m, far=100m。ハンドカメラだけ他より近くする。
        # 他カメラと同じ 0.07m だと、カメラから 6.2cm にある指の付け根が
        # クリップされて消え、指が途中から生えて見える (切断面の向こうが透ける)。
        #   カメラ -> proximal 0.062m / distal 0.083m / 指先 0.103m
        hand_camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))

        # ステレオ無効時も属性自体は必ず生えている状態にしておく (AttributeError 回避)。
        self.ros_camera_graph_l = None
        self.ros_camera_graph_r = None

        try:
            og.Controller.edit(
                {'graph_path': '/ros_controllers', 'evaluator_name': 'execution'},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ('OnImpulseEvent', 'omni.graph.action.OnImpulseEvent'),
                        ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                        ('PublishClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
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

            # ステレオ 2 本のグラフ (レンダープロダクト 1280x960 x2 込み) は既定では作らない。
            if _ENABLE_STEREO_CAMERAS:
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
                            ('cameraHelperRgb', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                            ('cameraHelperInfo', og_ros_camera_info_node()),
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
                            ('cameraHelperRgb.inputs:frameSkipCount',
                             _CAMERA_FRAME_SKIP),
                            ('cameraHelperRgb.inputs:type', 'rgb'),
                            ('cameraHelperInfo.inputs:frameId',
                             'head_l_stereo_camera_frame'),
                            (
                                'cameraHelperInfo.inputs:topicName',
                                topic_prefix + '/head_l_stereo_camera/camera_info',
                            ),
                            ('cameraHelperInfo.inputs:frameSkipCount',
                             _CAMERA_FRAME_SKIP),
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
                            ('cameraHelperRgb', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                            ('cameraHelperInfo', og_ros_camera_info_node()),
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
                            ('cameraHelperRgb.inputs:frameSkipCount',
                             _CAMERA_FRAME_SKIP),
                            ('cameraHelperRgb.inputs:type', 'rgb'),
                            ('cameraHelperInfo.inputs:frameId',
                             'head_r_stereo_camera_frame'),
                            (
                                'cameraHelperInfo.inputs:topicName',
                                topic_prefix + '/head_r_stereo_camera/camera_info',
                            ),
                            ('cameraHelperInfo.inputs:frameSkipCount',
                             _CAMERA_FRAME_SKIP),
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
                        ('cameraHelperRgb', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                        ('cameraHelperInfo', og_ros_camera_info_node()),
                        ('cameraHelperDepth', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                        ('cameraHelperDepthInfo', og_ros_camera_info_node()),
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
                        ('cameraHelperRgb.inputs:frameSkipCount',
                         _CAMERA_FRAME_SKIP),
                        ('cameraHelperRgb.inputs:type', 'rgb'),
                        ('cameraHelperInfo.inputs:frameId',
                         'head_rgbd_sensor_rgb_frame'),
                        (
                            'cameraHelperInfo.inputs:topicName',
                            topic_prefix + '/head_rgbd_sensor/rgb/camera_info',
                        ),
                        ('cameraHelperInfo.inputs:frameSkipCount',
                         _CAMERA_FRAME_SKIP),
                        ('cameraHelperDepth.inputs:frameId',
                         'head_rgbd_sensor_rgb_frame'),
                        (
                            'cameraHelperDepth.inputs:topicName',
                            topic_prefix + '/head_rgbd_sensor/depth_registered/image_rect_raw',
                        ),
                        ('cameraHelperDepth.inputs:frameSkipCount',
                         _CAMERA_FRAME_SKIP),
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
                        ('cameraHelperDepthInfo.inputs:frameSkipCount',
                         _CAMERA_FRAME_SKIP),
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
                        ('cameraHelperRgb', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                        ('cameraHelperInfo', og_ros_camera_info_node()),
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
                        ('cameraHelperRgb.inputs:frameSkipCount',
                         _CAMERA_FRAME_SKIP),
                        ('cameraHelperRgb.inputs:type', 'rgb'),
                        ('cameraHelperInfo.inputs:frameId', 'hand_camera_frame'),
                        (
                            'cameraHelperInfo.inputs:topicName',
                            topic_prefix + '/hand_camera/camera_info',
                        ),
                        ('cameraHelperInfo.inputs:frameSkipCount',
                         _CAMERA_FRAME_SKIP),
                    ],
                },
            )
        except Exception as e:
            raise e

        if _ENABLE_STEREO_CAMERAS:
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

        if _ENABLE_STEREO_CAMERAS:
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
        writer = rep.writers.get('RtxLidarROS2PublishPointCloud')
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
                    ('publishImu', 'isaacsim.ros2.bridge.ROS2PublishImu'),
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
                    ('publishLaserScan', 'isaacsim.ros2.bridge.ROS2PublishLaserScan'),
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
                        '/scan',
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
