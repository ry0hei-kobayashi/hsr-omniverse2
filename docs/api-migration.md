# Isaac Sim API 移行の記録 (ステップ4)

対象: Isaac Sim 5.1.0。`omni.isaac.*` 名前空間は 4.5 で非推奨になり、5.1 では `extsDeprecated/` に残っているだけで
6.0 で消える予定。記録日 2026-10-12。

## 1. 済み: `omni.isaac.core.*` → `isaacsim.*` (機械的置換、挙動不変)

| 旧 | 新 | 備考 |
|---|---|---|
| `omni.isaac.core.SimulationContext` | `isaacsim.core.api.SimulationContext` | |
| `omni.isaac.core.articulations.ArticulationView` | `isaacsim.core.prims.Articulation` | 引数 `prim_paths_expr` / `name` は同じ。`_metadata.joint_indices` (hsr.py の FT センサ読み出し) も同じ tensor view が裏にあるので互換 |
| `omni.isaac.core.prims.GeometryPrim` | `isaacsim.core.prims.SingleGeometryPrim` | 4.5 以降は view 版が `GeometryPrim`、単体が `SingleGeometryPrim`。`apply_physics_material` はそのまま |
| `omni.isaac.core.utils.{stage,viewports,prims,rotations,extensions,render_product}` | `isaacsim.core.utils.*` | 1:1 |
| `omni.isaac.core.utils.nucleus` | `isaacsim.storage.native` | hsr.py / launch_isaacsim.py では import だけで未使用だったので削除。sample-gpu-performance.py は `get_assets_root_path` に置換 |
| `omni.isaac.kit.SimulationApp` | `isaacsim.simulation_app.SimulationApp` | sample-*.py のみ |
| `omni.isaac.version.get_version` | `isaacsim.core.version.get_version` | sample-ycb-grasp.py のみ |

変更ファイル: `sim/hsr_omniverse2_sim/robot/hsr/robot.py`、`launch_isaacsim.py`、`construct_environment.py`、`people_spawn.py`、`furniture_spawn.py`、
`arena_cameras.py`、`env/scene_dressing/_common.py`、`sample-gpu-performance.py`、`sample-ycb-grasp.py`。

変更不要だったもの: Kit コマンド `RangeSensorCreateLidar` / `IsaacSensorCreateImuSensor` (5.1 の `isaacsim.sensors.physx` /
`isaacsim.sensors.physics` が同名で登録)、OmniGraph ノード型 (`isaacsim.ros2.bridge.ROS2*`、`isaacsim.core.nodes.*`、
`isaacsim.sensors.*`)、`isaacsim.core.api.materials.physics_material.PhysicsMaterial`、`isaacsim.sensors.physics.ContactSensor`。

## 2. 未着手: `omni.isaac.dynamic_control` (判断待ち)

### 現状
- 5.1 では `extsDeprecated/omni.isaac.dynamic_control` (2.0.7、cpython-311 .so + plugin) が動く。警告「deprecated since 4.5.0」。
  NVIDIA の移行表は「直接の代替なし」。6.0 で削除予定。
- 使用箇所: `sim/hsr_omniverse2_sim/robot/hsr/robot.py` (台車 kinematic 駆動・関節制御・剛体姿勢) と semu fork の `ros2_bridge.py` (関節読み書き)、
  `launch_isaacsim.py` (剛体取得)。メソッド 22 種 (`get_dof_state`, `set_dof_velocity_target`, `get_rigid_body_pose`,
  `set_rigid_body_pose`, `wake_up_articulation`, `find_articulation_dof`, `get_articulation_dof_properties` …) と
  定数/型 10 種 (`INVALID_HANDLE`, `Transform`, `JOINT_*`, `STATE_*`, `DofType`)。

### 選択肢
| 案 | 内容 | 工数目安 | リスク |
|---|---|---|---|
| A. 現状維持 | 5.1 の間は extsDeprecated で動く | 0 | 6.0 で動かなくなる |
| B. 互換シム `dc_compat.py` | `~/hsr-isaac-v2/scripts/dc_compat.py` (788 行、ユーザー作) を 3rdparty または scripts に置き、import を 1 行差し替える。中身は `isaacsim.core.prims.Articulation` / `RigidPrim` (omni.physics.tensors) で実装。使用中の 22 メソッドのうち **`is_simulating` 以外は実装済み** (未実装は NotImplementedError で止まる設計)。単位は旧 API 準拠 (stage units) で返すので呼び出し側の補正と二重にならない | 1 日 (組み込み 0.5 + 検証 0.5) | hsr-isaac-v2 では FJT 走行・把持で検証済みだが、hsr-omniverse2 の kinematic 台車 (`set_rigid_body_pose` 毎ステップ、anchor usd モード) と semu fork は未検証。`_metadata` 経由の DOF 順序が dynamic_control と一致する前提 |
| C. 新 API へ書き換え | 呼び出し側 ~116 箇所を `Articulation` / `RigidPrim` の tensor API に直接書き換え | 3〜5 日 | 書き換え間違いの機会が多い。hsr.py の台車駆動は実測で合わせ込んだコードなので回帰確認が重い |

### 推奨
B を Jazzy 化 (ステップ5) の後、別ブランチで試す。理由: 5.1 の間は A で困らず、B は差し替えと戻しが import 1 行で済むため
比較実験がしやすい。B に進むときは `is_simulating` を dc_compat に足す (semu fork の `RosAttribute.step` が使う)。

## 3. そのほか版依存の残り

- `hsr.py` の PhysX 球スイープ (`get_physx_scene_query_interface().sweep_sphere_all`、壁クランプ): 5.1 でそのまま動作
  (gpsr / tidyup の起動で `BASE_KINEMATIC_COLLISION=1` のまま確認)。
- `hsr.py` の `create_lidar_rtx` (未使用) は `RtxLidarROS2PublishPointCloud` writer を参照。RTX Lidar に切り替える場合は
  `isaacsim.sensors.rtx` の現行 API で書き直す。
- 起動ログの `omni.isaac.franka` / `omni.isaac.motion_generation` の非推奨警告は Isaac 側の拡張が出しているもので、
  このリポジトリのコードは呼んでいない。
