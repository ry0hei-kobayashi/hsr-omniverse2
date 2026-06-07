"""hsrc_ex (HSR-C) を Isaac Sim にスポーンするためのクラス。

元の scripts/hsr.py (HSR-B 用) は一切変更せず、その `hsr` クラスを継承して
HSR-B と異なる部分だけを上書きする:
  - 読み込む USD: usd/hsrb/hsrb4s.usd → usd/hsrc/hsrc1s.usd
  - 頭部RGBD / ハンドカメラのフレーム名 (Gemini336L / RealSense D405)
  - IMU の取付先 (hsrc_ex には base_imu_frame リンクが無いので base_link に付ける)

共通の制御ロジック (step, 各コントローラ, 関節処理 など) は hsr.py を
そのまま再利用するので、本ファイルには「違う部分」だけが書いてある。
どこを何に変えるか・なぜそうするかの根拠は docs/hsrc_ex_frame_mapping.md を参照。
"""
import os

import hsr as base

# Isaac Sim 関連の名前は元モジュール(hsr.py)が import 済みなので、そこから借りる。
# こうすると本ファイルで Isaac の重い import を書かずに済み、import 順の問題も避けられる。
og = base.og
omni = base.omni
stage = base.stage
set_targets = base.set_targets
UsdGeom = base.UsdGeom
og_ros_node = base.og_ros_node
is_ros2 = base.is_ros2


class hsr(base.hsr):
    """HSR-B 用 hsr クラスを継承し、hsrc_ex 固有の差分だけ上書きしたクラス。"""

    # hsrc_ex 固有のフレーム名 (HSR-B と異なる箇所のみ)。
    # 左右ステレオ・頭部RGBD の取付先リンクは HSR-B と同名なので上書き不要。
    RGBD_FRAME = 'head_rgbd_sensor_color_optical_frame'  # 頭部RGBD の frameId
    HAND_PRIM = 'hand_camera_link'                        # ハンドカメラ取付先(USDリンク名)
    HAND_FRAME = 'hand_camera_color_optical_frame'        # ハンドカメラ の frameId
    IMU_PRIM = 'base_link'      # IMU 取付先 (hsrc_ex に base_imu_frame リンクが無いため)
    IMU_FRAME = 'base_imu_frame'  # IMU の frameId (名前だけ従来どおり維持)

    def __init__(self, prefix='/hsrb', stage_path='/World', config=None):
        # base.hsr.__init__ は HSR-B の USD を読み込む実装になっている。
        # hsr.py は変更しない方針なので、__init__ の実行中だけ
        # add_reference_to_stage を一時的に差し替えて、ロボット本体を
        # hsrc の USD で読み込ませる。終わったら必ず元に戻す(finally)。
        hsrc_usd = self._resolve_hsrc_usd()
        _orig_add = base.stage.add_reference_to_stage

        def _add_with_hsrc(usd_path, prim_path, *args, **kwargs):
            # ロボット本体 (prim_path がスポーン先と一致) のときだけ hsrc USD に差し替える。
            # それ以外の参照読み込みは元の挙動のまま。
            if prim_path == stage_path + prefix:
                return _orig_add(hsrc_usd, prim_path, *args, **kwargs)
            return _orig_add(usd_path, prim_path, *args, **kwargs)

        base.stage.add_reference_to_stage = _add_with_hsrc
        try:
            super().__init__(prefix=prefix, stage_path=stage_path, config=config)
        finally:
            base.stage.add_reference_to_stage = _orig_add
        self.model = 'hsrc_ex'

    @staticmethod
    def _resolve_hsrc_usd():
        """hsrc の USD ファイルを配置に依らず見つける (hsr.py の探索と同じ流儀)。"""
        _here = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            os.path.join(_here, 'usd', 'hsrc', 'hsrc1s.usd'),
            os.path.join(_here, '..', 'usd', 'hsrc', 'hsrc1s.usd'),
            '/app/usd/hsrc/hsrc1s.usd',
        ]
        return next((p for p in candidates if os.path.exists(p)), candidates[0])

    def create_cameras(self) -> None:
        # 元 hsr.py の create_cameras をベースに、hsrc_ex で異なるフレーム名だけ差し替えたもの。
        # 左右ステレオ(head_l/r_stereo_*) は HSR-B と同じなので変更していない。
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

        # 頭部RGBD の取付先リンク (head_rgbd_sensor_link) は HSR-B と同名なのでそのまま。
        rgbd_camera_prim = UsdGeom.Camera(
            omni.usd
            .get_context()
            .get_stage()
            .DefinePrim(self.stage_path + self.prefix + '/head_rgbd_sensor_link/Camera', 'Camera')
        )
        xform_api = UsdGeom.XformCommonAPI(rgbd_camera_prim)
        # hsrc_ex の head_rgbd_sensor_link (Gemini336L) は x-forward リンクで、
        # hsrb (Xtion, リンク自体が光学向き) とは取付向きが異なる。HSR-B の
        # (180,0,0) を流用すると画像が90度回るため、標準ROS光学規約に合わせる:
        # camera_local = rpy(-90,0,-90)*Rx(180) = XYZオイラー(90,0,-90)。
        xform_api.SetRotate(
            (90, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        rgbd_camera_prim.GetHorizontalApertureAttr().Set(640 * 0.003)
        rgbd_camera_prim.GetVerticalApertureAttr().Set(480 * 0.003)
        rgbd_camera_prim.GetProjectionAttr().Set('perspective')
        rgbd_camera_prim.GetFocalLengthAttr().Set(554.382712823 * 0.003)
        rgbd_camera_prim.GetFocusDistanceAttr().Set(400)

        # ハンドカメラの取付先は hsrc_ex では hand_camera_link (hand_camera_frame は無い)。
        hand_camera_prim = UsdGeom.Camera(
            omni.usd
            .get_context()
            .get_stage()
            .DefinePrim(
                self.stage_path + self.prefix + '/' + self.HAND_PRIM + '/Camera', 'Camera'
            )
        )
        xform_api = UsdGeom.XformCommonAPI(hand_camera_prim)
        # hand_camera_link (D405) も x-forward リンク(55度下向き)。同様に
        # 標準ROS光学規約へ。XYZオイラー(90,0,-90)。
        xform_api.SetRotate(
            (90, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
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
                        # hsrc_ex の頭部RGBD(Gemini)は color 系の光学フレームを使う。
                        ('cameraHelperRgb.inputs:frameId',
                         self.RGBD_FRAME),
                        (
                            'cameraHelperRgb.inputs:topicName',
                            topic_prefix + '/head_rgbd_sensor/rgb/image_rect_color',
                        ),
                        ('cameraHelperRgb.inputs:type', 'rgb'),
                        ('cameraHelperInfo.inputs:frameId',
                         self.RGBD_FRAME),
                        (
                            'cameraHelperInfo.inputs:topicName',
                            topic_prefix + '/head_rgbd_sensor/rgb/camera_info',
                        ),
                        ('cameraHelperInfo.inputs:type', 'camera_info'),
                        ('cameraHelperDepth.inputs:frameId',
                         self.RGBD_FRAME),
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
                         self.RGBD_FRAME),
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
                        # hsrc_ex のハンドカメラ(D405)は color 系の光学フレームを使う。
                        ('cameraHelperRgb.inputs:frameId', self.HAND_FRAME),
                        (
                            'cameraHelperRgb.inputs:topicName',
                            topic_prefix + '/hand_camera/image_raw',
                        ),
                        ('cameraHelperRgb.inputs:type', 'rgb'),
                        ('cameraHelperInfo.inputs:frameId', self.HAND_FRAME),
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
            target_prim_paths=[self.stage_path + self.prefix +
                               '/' + self.HAND_PRIM + '/Camera'],
        )

        og.Controller.evaluate_sync(self.ros_camera_graph_l)
        og.Controller.evaluate_sync(self.ros_camera_graph_r)
        og.Controller.evaluate_sync(self.ros_camera_graph_rgbd)
        og.Controller.evaluate_sync(self.ros_camera_graph_hand)

    def create_imu(self) -> None:
        # 元 hsr.py の create_imu をベースに、IMU の取付先だけ base_link に変えたもの。
        # hsrc_ex の URDF には base_imu_frame リンクが無いため (gazebo 参照の残骸のみ)。
        # frameId は従来どおり base_imu_frame のまま維持する。
        _, sensor = omni.kit.commands.execute(
            'IsaacSensorCreateImuSensor',
            path=self.stage_path + self.prefix + '/' + self.IMU_PRIM + '/Imu_Sensor',
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
                    ('publishImu.inputs:frameId', self.IMU_FRAME),
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
            target_prim_paths=[self.stage_path + self.prefix +
                               '/' + self.IMU_PRIM + '/Imu_Sensor'],
        )
