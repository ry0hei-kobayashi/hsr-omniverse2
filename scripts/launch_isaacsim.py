# Copyright (c) 2023, Toyota Motor Corporation
# Copyright (c) 2023, MID Academic Promotions, Inc.
# All rights reserved.

# from isaacsim.simulation_app import SimulationApp
#
# kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
# kit.set_setting("/app/extensions/installUntrustedExtensions", True)

import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from isaacsim.simulation_app import SimulationApp

kit = SimulationApp({
    'renderer': 'RayTracedLighting',
    'headless': False,
    'extra_args': [
        '--/app/extensions/excluded/0=isaacsim.asset.importer.urdf',
        '--/app/extensions/excluded/1=isaacsim.ros2.urdf',
    ],
})
kit.set_setting('/app/extensions/installUntrustedExtensions', True)

import omni.kit.commands
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.version import get_version
from isaacsim.sensors.physics import ContactSensor
from isaacsim.storage.native import get_assets_root_path
from omni.isaac.core import SimulationContext
from omni.isaac.core.prims import GeometryPrim
from omni.isaac.core.utils import nucleus, stage, viewports
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.physx import get_physx_simulation_interface
from omni.physx.scripts import utils as physx_utils
from pxr import (Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom,
                 UsdPhysics)
from tmc_wrs_gazebo_worlds import randomizer

import hsr
import construct_environment
import object_placement
import people_spawn


# from omni.isaac.core.materials.physics_material import PhysicsMaterial


# from omni.isaac.core import SimulationContext
# from omni.isaac.core.utils import viewports, stage
# from omni.isaac.core.utils.prims import create_prim
# from omni.isaac.core.utils.rotations import euler_angles_to_quat
# import omni.kit.commands


# from isaacsim.sensors.physics import _sensor


# from pxr import Sdf, Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema, PhysicsSchemaTools
# from omni.physx import get_physx_simulation_interface
# from omni.isaac.sensor import ContactSensor
# from omni.isaac.sensor import _sensor
# from omni.isaac.core.materials.physics_material import PhysicsMaterial


is_ros2 = False
try:
    import rclpy
    from gazebo_msgs.srv import GetModelState, GetWorldProperties

    is_ros2 = True
except ImportError:
    import rospy
    from gazebo_msgs.srv import (GetModelState, GetModelStateResponse,
                                 GetWorldProperties,
                                 GetWorldPropertiesResponse)

try:
    import rosgraph

    if not rosgraph.is_master_online():
        print('Please run roscore before executing this script')
        kit.close()
        exit()
except ImportError:
    pass

viewports.set_camera_view(eye=np.array(
    [3.7, 1.7, 5.0]), target=np.array([0, 0, 0]))

create_prim(
    '/World/Light_1',
    'SphereLight',
    position=np.array([2.0, 0.0, 5.0]),
    attributes={'inputs:radius': 0.01, 'inputs:intensity': 5e4,
                'inputs:color': (1.0, 1.0, 1.0)},
)
create_prim(
    '/World/Light_2',
    'SphereLight',
    position=np.array([-2.0, 0.0, 5.0]),
    attributes={'inputs:radius': 0.01, 'inputs:intensity': 5e4,
                'inputs:color': (1.0, 1.0, 1.0)},
)


assets_root_path = get_assets_root_path()
if assets_root_path is None:
    raise RuntimeError('Could not find Isaac Sim assets root')


# Loading the simple_room environment
# assets_root_path = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/" + get_version()[0]
BACKGROUND_STAGE_PATH = '/background'
# BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"
# BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
# BACKGROUND_USD_PATH = "/Isaac/Environments/Hospital/hospital.usd"
# BACKGROUND_USD_PATH = "/Isaac/Environments/Office/office.usd"
BACKGROUND_USD_PATH = '/Isaac/Environments/Grid/default_environment.usd'

stage.add_reference_to_stage(
    assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

# adjust friction of the floor
floor_material = PhysicsMaterial(
    prim_path='/Floor', static_friction=60.0, dynamic_friction=60.0)

# omni.kit.commands.execute('BindMaterialExt',
#                            material_path='/Floor',
#                            prim_path=[BACKGROUND_STAGE_PATH + '/GroundPlane/CollisionPlane'],
#                            strength=['weakerThanDescendants'],
#                            material_purpose='physics')


floor_material = PhysicsMaterial(
    prim_path='/World/PhysicsMaterials/FloorMaterial',
    static_friction=60.0,
    dynamic_friction=60.0,
)

ground_prim = GeometryPrim(
    prim_path=BACKGROUND_STAGE_PATH + '/GroundPlane/CollisionPlane')
ground_prim.apply_physics_material(
    floor_material, weaker_than_descendants=True)

model_names = []
repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
model_root = os.path.join(repo_root, 'usd', 'wrs_models')
if not os.path.exists(model_root):
    model_root = '/app/usd/wrs_models'

# ============================================================
# タスク選択 (TASK 環境変数)
# ============================================================
# `make ros2 dev run TASK=hri` のように指定すると、configs/tasks/<TASK>/ の
# 設定 (world / placement / dressing) を使う。TASK 未指定なら従来どおり
# configs 直下の既定を使う (後方互換)。
#   - task.yaml ... world のファイル名 と dressing の選択 (preset/lighting)
#   - placement.yaml (任意) ... robot/objects/people。無ければ共通の既定。
_task = os.environ.get('TASK', '').strip()
_task_world_name = None        # task.yaml の world: (worlds/ 内のファイル名)
_task_placement_path = None    # None なら configs/placement.yaml (各ローダーの既定)
_task_dressing_preset = None   # None なら dressing.yaml の defaults
_task_dressing_lighting = None
if _task:
    _task_dir = next(
        (d for d in (os.path.join('/app/configs/tasks', _task),
                     os.path.join(repo_root, 'configs', 'tasks', _task))
         if os.path.isdir(d)),
        None)
    if _task_dir is None:
        raise FileNotFoundError(
            f"TASK='{_task}' のフォルダが見つかりません。"
            f"configs/tasks/{_task}/ を作ってください。")
    print(f"[task] selected TASK='{_task}' dir={_task_dir}")

    # task.yaml (world と dressing の選択) を読む。
    _task_yaml = os.path.join(_task_dir, 'task.yaml')
    if os.path.isfile(_task_yaml):
        with open(_task_yaml) as _f:
            _tcfg = yaml.safe_load(_f) or {}
        _task_world_name = _tcfg.get('world')
        _dsel = _tcfg.get('dressing') or {}
        _task_dressing_preset = _dsel.get('preset')
        _task_dressing_lighting = _dsel.get('lighting')
    else:
        print(f"[task] WARNING: {_task_yaml} が無いので world/dressing は既定を使います。")

    # placement.yaml はタスクフォルダにあればそれを優先 (無ければ共通の既定)。
    _p = os.path.join(_task_dir, 'placement.yaml')
    if os.path.isfile(_p):
        _task_placement_path = _p
        print(f"[task] placement: {_p}")
    else:
        print("[task] placement: タスク個別が無いので共通の configs/placement.yaml を使用")

# Extract poses of objects from the world file
# タスクで world が指定されていれば、それを候補の先頭に置く。
if _task_world_name:
    # タスクが world を明示している場合は、それが見つからなければ
    # 既定 world に黙って落ちず、エラーで止める (typo を見逃さないため)。
    _task_world_candidates = [
        os.path.join('/app/worlds', _task_world_name),
        os.path.join(repo_root, 'worlds', _task_world_name),
    ]
    world_file = next(
        (p for p in _task_world_candidates if os.path.exists(p)), None)
    if world_file is None:
        raise FileNotFoundError(
            f"TASK='{_task}' の world '{_task_world_name}' が worlds/ に見つかりません。"
            f"task.yaml の world: を worlds/ 内の正しいファイル名にしてください "
            f"(探した場所: {_task_world_candidates})。")
else:
    # タスク未指定 (または task.yaml に world: なし) のときは従来の候補から探す。
    world_candidates = [
        '/app/worlds/rc26_3330.world',
        os.path.join(repo_root, 'worlds', 'rc26_3330.world'),
        '/app/worlds/env_furniture_rcj26_pre2.world',
        os.path.join(repo_root, 'worlds', 'env_furniture_rcj26_pre2.world'),
        '/app/worlds/rcj26_pre2.world',
        os.path.join(repo_root, 'worlds', 'rcj26_pre2.world'),
        os.path.join(repo_root, 'rcj26_pre2.world'),
    ]
    world_file = next(
        (p for p in world_candidates if os.path.exists(p)), None)
    if world_file is None:
        if is_ros2:
            world_file = (
                '/ws/install/tmc_wrs_gazebo_worlds/share/tmc_wrs_gazebo_worlds/worlds/wrs2020_knob.world'
            )
        else:
            world_file = '/opt/ros/noetic/share/tmc_wrs_gazebo_worlds/worlds/wrs2020_knob.world'
print(f'Loading world file: {world_file}')
if not os.path.exists(world_file):
    raise FileNotFoundError(f'World file not found: {world_file}')
tree = ET.parse(world_file)
root = tree.getroot()
for i in root.findall('world/include'):
    model_name = i.find('name').text
    model_uri = i.find('uri').text
    (x, y, z, er, ep, ey) = [float(n) for n in i.find('pose').text.split(' ')]
    stage_path = f'/{model_name}'
    model_path = os.path.join(
        model_root, model_uri.replace('model://', ''), 'model.usd'
    )
    if model_uri == 'model://unit_box' and not os.path.exists(model_path):
        scale_tag = i.find('scale')
        if scale_tag is None:
            size = np.array([1.0, 1.0, 1.0])
        else:
            size = np.array([float(n) for n in scale_tag.text.split(' ')])
        size = np.maximum(size, 1e-4)
        orientation = euler_angles_to_quat([er, ep, ey])
        create_prim(
            prim_path=stage_path,
            prim_type='Cube',
            translation=[x, y, z],
            orientation=orientation,
            scale=size * 0.5,
        )
        # 家具・壁の Cube に当たり判定 (collider) を付ける。
        # RigidBodyAPI は付けないので「動かない固い箱」になり、この上に置いた
        # 物体 (YCB 等) が天板に乗って止まる (collider が無いとすり抜けて落ちる)。
        _cube_prim = omni.usd.get_context().get_stage().GetPrimAtPath(stage_path)
        physx_utils.setCollider(_cube_prim, approximationShape='none')
        model_names.append(model_name)
    if not os.path.exists(model_path):
        continue
    create_prim(
        prim_path=stage_path,
        prim_type='Xform',
        translation=[x, y, z],
        orientation=euler_angles_to_quat([er, ep, ey]),
    )
    stage.add_reference_to_stage(model_path, Sdf.Path(stage_path))
    if i.find('static') is not None:
        # Create fixed joint between the world if the object is static
        root_joint = UsdPhysics.FixedJoint.Define(
            omni.usd.get_context().get_stage(), stage_path + '/root_joint'
        )
        root_joint.CreateBody1Rel().SetTargets([stage_path + '/link'])
        root_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
        root_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        root_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        root_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    model_names.append(model_name)


def _subtree_has_rigid_body(prim):
    """prim とその子孫のどこかに既に剛体 (RigidBodyAPI) が付いているか調べる。

    YCB など model.usd 自身に物理が入っているモデルは True を返す。
    物理が無い自作モデル (my_models) は False。
    """
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            return True
    return False


def drop_object(gazebo_name, name, x, y, z, yaw=0.0, roll=0.0, pitch=0.0):
    global model_names
    # USD の prim パスは "/" が階層区切りになるため、name に相対パス
    # (rc26_practice_day_1/drink/led 等) が含まれると壊れる。"-" と "/" を
    # まとめて "_" に置換し、1 階層の安全な prim 名にする。
    safe_name = gazebo_name.replace("-", "_").replace("/", "_")
    stage_path = f'/{safe_name}'
    # name は次の 2 通りの書き方を許す:
    #   (A) usd/ からの相対パス  例: 'rc26_practice_day_1/drink/led'
    #       -> usd/rc26_practice_day_1/drink/led/model.usd を直接指す。
    #          フォルダ階層まで明示するので、物体名の重複が起きない。
    #   (B) 物体フォルダ名だけ    例: 'ycb_011_banana', 'led'
    #       -> 従来どおり my_models / wrs_models を探す (後方互換)。
    model_candidates = [
        os.path.join(repo_root, 'usd', name, 'model.usd'),          # (A)
        os.path.join(repo_root, 'usd', 'my_models', name, 'model.usd'),
        os.path.join(model_root, name, 'model.usd'),
        '/app/usd/' + name + '/model.usd',                          # (A)
        '/app/usd/my_models/' + name + '/model.usd',
        '/app/usd/wrs_models/' + name + '/model.usd',
    ]
    model_path = next((p for p in model_candidates if os.path.exists(p)), None)
    if model_path is None:
        print(f'Model not found for {name}: tried {model_candidates}')
        return None
    create_prim(
        prim_path=stage_path,
        prim_type='Xform',
        translation=[x, y, z],
        orientation=euler_angles_to_quat([roll, pitch, yaw]),
    )
    stage.add_reference_to_stage(model_path, Sdf.Path(stage_path))

    # Object Capture (iPhone 撮影) 製などは upAxis=Y で作られており、Z-up の
    # 世界ではそのままだと横倒しになる。参照の「後」に rotateX(90) を足して
    # 立たせる。create_prim が [translate, orient] を設定済みなので、ここで足すと
    # 順序が [translate, orient, rotateX] になり、ジオメトリにはまず rotateX
    # (Y-up→Z-up) → 次に向き → 最後に位置、の順で効く。
    # ※ モデルのルートに焼き込んだ回転は create_prim の設定に上書きされて効かない
    #   ため、必ずこのコード側で足す (people_spawn.py が人を立たせるのと同じ手法)。
    # ※ Z-up の YCB / wrs_models には足さない (upAxis を見て判定)。
    try:
        _src_stage = Usd.Stage.Open(model_path)
        _is_y_up = UsdGeom.GetStageUpAxis(_src_stage) == UsdGeom.Tokens.y
    except Exception:
        _is_y_up = False
    if _is_y_up:
        _prim = omni.usd.get_context().get_stage().GetPrimAtPath(stage_path)
        UsdGeom.Xformable(_prim).AddRotateXOp().Set(90.0)

    # 物理 (衝突判定 + 重力) を保証する。
    # YCB などは model.usd 自身に剛体が入っているのでそのまま使う。
    # led のように物理を持たない自作モデルには、ここで剛体 + 当たり判定を付けて、
    # 他の物体と同じように天板へ落ちて乗るようにする (浮いたままにならない)。
    dropped_prim = omni.usd.get_context().get_stage().GetPrimAtPath(stage_path)
    if not _subtree_has_rigid_body(dropped_prim):
        # convexHull = 物体の外形を凸形状で近似した当たり判定。
        # (動く物体の標準。三角メッシュ 'none' は静止物専用で落下に使えない)
        physx_utils.setRigidBody(dropped_prim, 'convexHull', False)

    model_names.append(gazebo_name)
    return model_path


# ランダム配置 (WRS 競技の出題) は使わず、configs/placement.yaml の指定に
# 従って家具の上に物体を配置する。ランダムに戻したいときは下を有効化:
#   randomizer.generate_wrs_task(drop_func=drop_object)
object_placement.apply_placements(world_file, drop_object,
                                  config_path=_task_placement_path)

# placement.yaml の people: セクションに従って「人」を配置する。
# people が空 (デフォルト) のときは何も置かず、既存の動作は変わらない。
# 戻り値: (配置人数, ループ開始秒, ループ終了秒)。人数はメインループで
# kit.update() を回すか判断するのに、開始/終了秒はアニメをループ再生させる
# タイムラインの再生区間 (start_time / end_time) に使う。
# config_path=None のときは各ローダーが共通の configs/placement.yaml を読む。
_num_people, _people_loop_start, _people_loop_end = people_spawn.spawn_people(
    assets_root_path, kit, config_path=_task_placement_path)

# 独自オブジェクト (usd/my_models) の配置は使わない。
# 配置は configs/placement.yaml の ycb のみ。戻したいときは下を有効化:
# # アルボナース
# drop_object(
#     gazebo_name='my_object',
#     name='my_object',
#     x=-0.26,
#     y=-0.79097,
#     z=-1.66326,
#     roll=math.pi / 2,
# )
# # hma宣伝ボード
# drop_object(
#     gazebo_name='hma_display_board',
#     name='hma_display_board',
#     x=0.8,
#     y=0.5,
#     z=0.5,
#     roll=math.pi / 2,
# )

# HSR の初期スポーン位置 (map 座標 = world 座標) は configs/placement.yaml の
# robot: セクションで定義する (robot/objects/people をまとめた設定ファイル)。
# 注意: env_furniture の operator_position は「人」の位置であって、ロボットの位置ではない。
_robot_spawn = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}  # 設定ファイルが無いときのフォールバック
# タスク個別の placement.yaml があればそれを優先 (無ければ共通の既定)。
_spawn_candidates = ([_task_placement_path] if _task_placement_path else []) + [
    '/app/configs/placement.yaml',
    os.path.join(repo_root, 'configs', 'placement.yaml'),
]
_spawn_path = next((p for p in _spawn_candidates if os.path.exists(p)), None)
if _spawn_path is not None:
    with open(_spawn_path) as _f:
        _cfg = yaml.safe_load(_f) or {}
    _robot_cfg = _cfg.get('robot') or {}   # robot: セクションを取り出す
    for _k in ('x', 'y', 'yaw'):
        if _robot_cfg.get(_k) is not None:
            try:
                _robot_spawn[_k] = float(_robot_cfg[_k])
            except (TypeError, ValueError):
                # 数値でない (例: 小数点を ',' で書いた) 場合でも sim を落とさず継続。
                print(f"[hsr] WARNING: placement.yaml の robot.{_k}={_robot_cfg[_k]!r} は"
                      f"数値として読めません。フォールバック {_robot_spawn[_k]} を使用 "
                      f"(小数点は '.' で書いてください)。")
    print(f'[hsr] spawn from {_spawn_path} (robot:): {_robot_spawn}')
else:
    print(f'[hsr] placement.yaml が無いのでフォールバック値を使用: {_robot_spawn}')

hsr_stage_path = '/hsrb'
create_prim(
    prim_path=hsr_stage_path,
    prim_type='Xform',
    translation=[_robot_spawn['x'], _robot_spawn['y'], 0],
    orientation=euler_angles_to_quat([0, 0, _robot_spawn['yaw']]),
)

_hsr = hsr.hsr(stage_path=hsr_stage_path)
model_names.append('hsrb')

if is_ros2:
    import std_msgs.msg

    collision_detect_pub = _hsr.ros2node.create_publisher(
        std_msgs.msg.Bool,
        '/undesired_contact_detector/detect',
        qos_profile=rclpy.qos.qos_profile_system_default,
    )
else:
    import std_msgs.msg

    collision_detect_pub = rospy.Publisher(
        '/undesired_contact_detector/detect', std_msgs.msg.Bool, queue_size=10
    )

contact_links = [
    '/hsrb/hsrb/base_link/collisions',
    '/hsrb/hsrb/base_f_bumper_link/collisions',
    '/hsrb/hsrb/base_b_bumper_link/collisions',
]

contact_sensors = []
stage_handle = omni.usd.get_context().get_stage()
for i in range(len(contact_links)):
    contact_report_api = PhysxSchema.PhysxContactReportAPI.Apply(
        stage_handle.GetPrimAtPath(contact_links[i])
    )
    contact_report_api.CreateThresholdAttr(0.0)
    contact_sensors.append(
        ContactSensor(
            prim_path=f'{contact_links[i]}/Contact_Sensor',
            name='Contact_Sensor',
            frequency=10,
            min_threshold=0,
            radius=-1,
        )
    )


actor_to_body_name_cache = {}
prev_contact = None


def contact_report_event(ch, cd):
    global prev_contact
    for c in ch:
        try:
            body1 = actor_to_body_name_cache[c.actor1]
        except KeyError:
            body1 = str(PhysicsSchemaTools.intToSdfPath(
                c.actor1)).split('/')[1]
            actor_to_body_name_cache[c.actor1] = body1
        if body1 != 'background':
            if prev_contact != body1:
                print(f'Contact {body1}')
                prev_contact = body1
        if body1 == 'wrc_frame' or body1.startswith('task2a_'):
            collision_detect_pub.publish(std_msgs.msg.Bool(data=True))


# this variable is unused, but it is required to continue the subscription
_contact_report_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(
    contact_report_event
)

# Start simulation
kit.update()
simulation_context = SimulationContext(stage_units_in_meters=1.0)
kit.update()
_hsr.onsimulationstart(simulation_context)
simulation_context.initialize_physics()
omni.timeline.get_timeline_interface().play()

# 人を配置したときだけアニメーションをループ再生する設定にする。
# (人が居ないときはタイムラインに触れず、従来どおりの挙動を保つ。)
# set_looping だけだと「どこで折り返すか」が分からずアニメが最後のポーズで
# 止まってしまう。ループ周期 (_people_loop_duration) を end_time に設定して
# はじめてループする (復元ガイドの知見)。
# この周期は people_spawn 側で「一番短いクリップ」に決めている。タイムラインは
# シーンに 1 本だけで全員が共有するため、こうしないと短いクリップの人が
# 長いクリップの人を待つ間フリーズしてしまうため (詳細は people_spawn.py)。
if _num_people > 0:
    _timeline = omni.timeline.get_timeline_interface()
    # 再生区間を [開始秒, 終了秒] に絞ってループさせる。
    # loop_window で「手を上げて振っている区間」だけを指定すると、手を下ろす
    # 部分が再生範囲から外れ、上げっぱなしで振り続けているように見える。
    if _people_loop_end > 0.0:
        _timeline.set_start_time(_people_loop_start)
        _timeline.set_end_time(_people_loop_end)
        # 再生ヘッドを区間の先頭に置いてから始める (区間外から始まらないように)。
        _timeline.set_current_time(_people_loop_start)
    _timeline.set_looping(True)
    print(f'[people] timeline looping on for {_num_people} character(s), '
          f'window={_people_loop_start:.2f}s..{_people_loop_end:.2f}s')

# ラボ環境テクスチャ (床 + 周囲背景 + 照明) を適用。
# timeline.play() の "後" でないと PhysX セットアップを壊すので注意。
# 床・背景幕 (周囲4枚の壁) を world の家具・壁の広がりに合わせて
# 自動でサイズ・中心を決める。world が原点からずれていても正しく囲める。
# タスクで dressing の preset/lighting を選んでいれば、それを渡す
# (未指定なら apply_lab_dressing 側が dressing.yaml の defaults を使う)。
_dress_kwargs = {}
if _task_dressing_preset:
    _dress_kwargs['preset'] = _task_dressing_preset
if _task_dressing_lighting:
    _dress_kwargs['lighting'] = _task_dressing_lighting
_bounds = object_placement.world_xy_bounds(world_file)
if _bounds is not None:
    _min_x, _max_x, _min_y, _max_y = _bounds
    _margin = 0.5  # 外周から壁を少し外に出す余白 (m)
    _room_size = max(_max_x - _min_x, _max_y - _min_y) + 2.0 * _margin
    _center_x = (_min_x + _max_x) / 2.0
    _center_y = (_min_y + _max_y) / 2.0
    print(f'[dressing] world bounds -> room_size={_room_size:.2f} '
          f'center=({_center_x:.2f}, {_center_y:.2f})')
    construct_environment.apply_lab_dressing(
        room_size=_room_size,
        center_x=_center_x,
        center_y=_center_y,
        **_dress_kwargs,
    )
else:
    construct_environment.apply_lab_dressing(**_dress_kwargs)
for _ in range(3):
    kit.update()


# simulate gazebo ros APIs required for task evaluators
def get_xform(stage, model_name):
    try:
        name = model_name.replace('::link', '').replace('-', '_')
        prim = stage.GetPrimAtPath(f'/{name}/link')
        if not prim.IsValid():
            prim = stage.GetPrimAtPath(f'/{name}/body')
        if not prim.IsValid():
            prim = stage.GetPrimAtPath(f'/{name}/hsrb/base_footprint')
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    except:
        # print(f'Failed to get xform for {model_name}')
        return Gf.Matrix4d()


if is_ros2:

    def handle_get_world_properties_ros2(req, ret):
        ret.model_names = model_names
        ret.success = True
        return ret

    def handle_get_model_state_ros2(req, ret):
        stage = omni.usd.get_context().get_stage()
        objxform = get_xform(stage, req.model_name)
        refxform = get_xform(stage, req.relative_entity_name)
        relpose = objxform * refxform.GetInverse()
        translation = relpose.ExtractTranslation()
        rotation = relpose.GetOrthonormalized().ExtractRotationQuat()
        rotation_imaginary = rotation.GetImaginary()
        # create response
        ret.header.frame_id = req.relative_entity_name
        ret.pose.position.x = translation[0]
        ret.pose.position.y = translation[1]
        ret.pose.position.z = translation[2]
        ret.pose.orientation.x = rotation_imaginary[0]
        ret.pose.orientation.y = rotation_imaginary[1]
        ret.pose.orientation.z = rotation_imaginary[2]
        ret.pose.orientation.w = rotation.GetReal()
        ret.success = True
        return ret

    _hsr.ros2node.create_service(
        GetWorldProperties,
        '/gazebo/get_world_properties',
        handle_get_world_properties_ros2,
        qos_profile=rclpy.qos.qos_profile_services_default,
    )
    _hsr.ros2node.create_service(
        GetModelState,
        '/gazebo/get_model_state',
        handle_get_model_state_ros2,
        qos_profile=rclpy.qos.qos_profile_services_default,
    )
else:

    def handle_get_world_properties(req):
        ret = GetWorldPropertiesResponse()
        ret.model_names = model_names
        ret.success = True
        return ret

    def handle_get_model_state(req):
        stage = omni.usd.get_context().get_stage()
        objxform = get_xform(stage, req.model_name)
        refxform = get_xform(stage, req.relative_entity_name)
        relpose = objxform * refxform.GetInverse()
        translation = relpose.ExtractTranslation()
        rotation = relpose.GetOrthonormalized().ExtractRotationQuat()
        rotation_imaginary = rotation.GetImaginary()
        # create response
        ret = GetModelStateResponse()
        ret.header.frame_id = req.relative_entity_name
        ret.pose.position.x = translation[0]
        ret.pose.position.y = translation[1]
        ret.pose.position.z = translation[2]
        ret.pose.orientation.x = rotation_imaginary[0]
        ret.pose.orientation.y = rotation_imaginary[1]
        ret.pose.orientation.z = rotation_imaginary[2]
        ret.pose.orientation.w = rotation.GetReal()
        ret.success = True
        return ret

    rospy.Service('/gazebo/get_world_properties',
                  GetWorldProperties, handle_get_world_properties)
    rospy.Service('/gazebo/get_model_state',
                  GetModelState, handle_get_model_state)

# disable showing lidar beam
_lidar_path = '/hsrb/hsrb/base_range_sensor_link/Lidar'
_lidar_prim = omni.usd.get_context().get_stage().GetPrimAtPath(_lidar_path)
if _lidar_prim.IsValid():
    _draw_attr = _lidar_prim.GetAttribute('drawLines')
    if _draw_attr and _draw_attr.IsValid():
        _draw_attr.Set(False)
        print(f'[sample-ros] disable showing lidar beam: {_lidar_path}')

while kit.is_running():
    # Run with a fixed step size
    if _num_people > 0:
        # 人 (UsdSkel) のアニメーションは kit.update() を回さないと評価されない。
        # ただし step(render=True) は内部で描画するので、その後に kit.update()
        # を足すと「1 コマで 2 回描画」になりレンダラが不安定になる
        # (X 接続断・セグフォルトの原因)。そこで物理ステップは描画なし
        # (render=False) にし、描画とアニメ評価は kit.update() の 1 回に任せる。
        simulation_context.step(render=False)
        kit.update()
    else:
        # 人が居ないときは従来どおり (描画つき物理ステップのみ)。
        simulation_context.step(render=True)
    _hsr.step()

simulation_context.stop()
kit.close()
