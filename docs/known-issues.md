# 注意点・懸念点 (ステップ2: carrobo → hsr-omniverse2 移植 + Isaac 5.1 化)

記録日: 2026-10-11。各項目は「状況 / 影響 / 対処 or 判断待ち」で書く。解決したら該当項目を消すか「解決済み」に変える。

## 1. この機体では既定の DDS 設定でプロセス間探索が成立しない

- 状況: `make up`（`assets/cyclonedds.xml`、物理 NIC `enp3s0` に autodetermine + マルチキャスト）だと、
  同一ホスト上の別プロセスどうしが一切見えない。新規ノードから見える topic は自分の `/rosout` `/parameter_events` の 2 本だけ、
  `joint_state_publisher` が robot_description を待ち続ける、`safe_pose_changer` が
  `Communication with arm_trajectory_controller failed` を出し続ける。
- 原因 (推定・未確認): ホストで `ufw` が active。enp3s0 を経由してループバックしてくる DDS の UDP
  (SPDP 239.255.0.1 / unicast 139xx 番台 = domain 26) が INPUT で落とされている。
  `ip maddr` では 239.255.0.1 に 96 ソケットが参加しているのに受信できない。
- 対処: `make up localhost`（`assets/cyclonedds.localhost.xml`、lo 固定・unicast peer）なら
  107 topics / 59 nodes が即見える。**この機体での起動・検証は常に `localhost` modifier を付ける。**
- 判断待ち: ufw に UDP 7400-7500 / 13900-14000 と 239.255.0.1 を許可する規則を足せば既定設定でも動くはず。
  別 PC と通信する運用 (`pc` モード) が必要になった時点で試す。

## 2. `ros2` CLI (`ros2 topic list` 等) がコンテナ内で固まることがある

- 状況: `docker exec ... ros2 topic list` が daemon 経由でも `--no-daemon` でも返ってこないことがあった
  (1 の状態のとき)。rclpy で直接ノードを立てる Python スクリプトは動く。
- 対処: 確認は rclpy スクリプトで行う (`examples/` の形式)。`ros2 daemon stop` を先に打つ。
  1 を直せば CLI も戻る見込み。

## 3. 競技モードの録画は root 所有で出る

- 状況: Isaac コンテナは root で動く (ホストの `~/.hsr-omniverse/cache` を書くため)。
  `recordings/<timestamp>/arena.mp4` と `recordings/tune/` は root 所有になる。
- 影響: ホスト側で消すのに `sudo` が要る。
- 判断待ち: 非 root (uid 1234 isaac-sim) で動かし、キャッシュを named volume にする方式
  (`~/hsr-isaac-v2/docker-compose.yml` の形) に切り替えるか。切り替えるとキャッシュの置き場所が変わる。

## 4. Isaac 5.1 で非推奨 API を使い続けている

- `omni.isaac.dynamic_control` (2.0.7): `extsDeprecated/` に残っており動くが「deprecated since 4.5.0」、
  **6.0 で削除予定**。`scripts/hsr.py` に約 80 箇所、semu fork の `ros2_bridge.py` に定数参照 15 箇所。
- `omni.isaac.core.*` (22 import / 9 ファイル)、`omni.isaac.range_sensor` の `RangeSensorCreateLidar`、
  `omni.isaac.motion_generation` / `omni.isaac.franka` (起動ログに警告) も同様。
- 判断 (決定済み): ステップ2 ではそのまま使う。新 API (`isaacsim.core.*`, `omni.physics.tensors` 等) への移行と
  動作可否はリファクタ (ステップ4) で検討する。`~/hsr-isaac-v2/scripts/dc_compat.py`（dynamic_control 代替シム、
  未実装 API は NotImplementedError で止まる設計）が参考になる。

## 5. Isaac 5.1 起動ログの既知の警告・エラー

- `[isaacsim.sensors.physics.plugin] *** error: .../Contact_Sensor parent prim is missing contact report API, automatically adding ...`
  (base_link / 前後バンパー): 4.5 でも出ていたもの。自動で API が付くので実害は見えていない。
  必要なら hsrb USD 側 (`usd/hsrb`) に `PhysxContactReportAPI` を付けて消す。
- `[isaacsim.ros2.bridge.impl.camera_info_utils] Unsupported physical distortion model 'None'. Using plumb_bob ...` と
  `Forcing fy to fx`: 5.1 の `ROS2CameraInfoHelper` の仕様。camera_info は出ている (21.6 Hz)。
  実機の内部パラメータに合わせたい場合は hsrc_ex と同様に camera prim の焦点距離を調整する。
- `[omni.usd] Coding Error: in SetEditTarget ... Layer @metrics:UnitsAdjust-...` が大量に出る: 物体 USD の単位補正レイヤ由来。
  4.5 でも出ていたか未確認。実害は見えていない。
- `omni.timeline.plugin Deprecated: direct use of ITimeline callbacks`: 4 と同種。

## 6. 実時間比 (RTF) が約 0.7

- 状況: RTX A6000、gpsr タスク、GUI あり、ステレオ無し、`RENDER_EVERY_N_STEPS=2` で `/clock` 43 Hz (物理 60 Hz 設定)。
  カメラ系は 21.6 Hz (設計値 30 Hz × RTF)。
- 対処候補: `CAMERA_FRAME_SKIP=1` / `RENDER_EVERY_N_STEPS=3` (README 参照)、headless 化。
  4.5 時代の RTF と比較していないので「5.1 で遅くなった」かどうかは不明。

## 7. `hsr_hsrc_ex.py` は 5.1 + 新 hsr.py で未検証

- 状況: carrobo 由来の hsr.py (台車 kinematic 駆動・ブレーキ・アーム調整) を全部取り込んだが、
  `robot=hsrc_ex` (hsrc1s.usd) での起動は試していない。camera_info ノードの差し替えと
  `og_ros_camera_info_node` の alias 追加だけ行った。
- 対処: `make up localhost robot=hsrc_ex` で一度起動して、カメラ・把持・台車を確認する。

## 8. ROS1 依存の削除 (ステップ3、2026-10-12 完了)

- 削除したファイル: `3rdparty/OgnROS1Action*.py`、`env_docker/Dockerfile.isaacsim` (noetic)、`env_docker/Dockerfile.ros`、
  `env_docker/docker-compose.yml` (ROS1 スタック)、`env_docker/docker-compose-multigpu.yml` (ROS1 の Dockerfile に依存)、
  `launch/hsr.launch`、`scripts/sample-ros-softbody.py` (roscore 前提)。
- コードから消したもの: `HSR_ROS_VERSION` セレクタと `is_ros2` 分岐 (hsr.py 22 箇所、hsrc_ex 1、launch_isaacsim 3)、
  rospy / actionlib / `tf.transformations` 経路、`og_ros_node()` (OGN ノード名は `isaacsim.ros2.bridge.ROS2*` を直接記述)、
  `reset_world_matcher_helper.py` の ROS1 分岐、Makefile の `ros1` 選択。
- ステップ4 で名前も整理した: `env_docker/docker-compose.yml` (旧 docker-compose-ros2.yml)、`env_docker/Dockerfile.isaacsim`
  (旧 Dockerfile.isaacsim-ros2)、`assets/ros_entrypoint.ros2.sh` / `ros_entrypoint.isaacsim.sh`、`make <action>` (ros2 単語不要。
  `make ros2 up` も互換で通る)、`bkup/` 削除。
- `env_docker/Dockerfile.ycb-grasp` / `docker-compose-ycb-grasp.yml` は ROS1 ではないので残した (動作未確認)。
- 検証: `make up localhost TASK=gpsr` で起動し、FJT 移動・`/gripper_controller/grasp` (tmc_control_msgs)・IMU・wheel_odom・カメラ・scan を確認。Python エラーなし。

## 9. 外部由来コードの出所・ライセンス

- `3rdparty/semu.robotics.ros2_bridge` (submodule): Toni-SM / MIT。fork ブランチ `isaac5.1-humble` は上流と同じ拡張構成のまま
  Python 3.11 / Isaac 5.1 向けに作り直したもの (`tools/build.sh`: control_msgs + add_on_msgs の py3.11 ビルド、
  Cython 版 `_ros2_bridge.cpython-311-*.so`、`exts/` パッケージ)。上流と同じく成果物 (約 11 MB) をコミットする方針。
  `add_on_msgs` の .srv は上流 README から復元 (`src/add_on_msgs`)。ヘッドレス Isaac 5.1 で拡張をロードし
  `/get_prim_attribute` の get/set が通ることを確認済み。hsr-omniverse2 自体は拡張ではなく `ros2_bridge.py` をソース import する
  (速度・安定性の判断。`docs/known-issues.md` 作成時点)。
  fork ブランチは push 済み (`692def7` "update for humble and py3.11"、1 commit)。hsr-omniverse2 の gitlink と
  `.gitmodules` の `branch = isaac5.1-humble` は staged 済み (未 commit)。
  `tools/build.sh` の注意: Isaac イメージは `ACCEPT_EULA=Y` と `--entrypoint bash` が必要 (既に script に組み込み済み)。
- msgbuild ステージで clone するもの: `ros-controls/control_msgs` (humble, BSD-3)、`hsr-project/tmc_realtime_control` (humble)、
  `ros-simulation/gazebo_ros_pkgs` の `gazebo_msgs` (ブランチ `ros2`、BSD-3)、`DLu/tf_transformations` (main、BSD-3)、
  `ry0hei-kobayashi/tmc_wrs_gazebo` の `tmc_wrs_gazebo_worlds` (Python 部分のみコピー。`docs/tmc_wrs_gazebo_worlds-build-note.md`)。
  LICENSE はイメージ内 `/app/ros_msgs/licenses/` に同梱。ブランチ名での clone なので再現性は低い (commit で固定したければ Dockerfile に SHA を書く)。
- `assets/joint_limits-smooth.yaml` / `assets/moveit-no-octomap.rviz`: `hsrb_moveit_config` (hsr-project) の設定を改変したもの。
  bind mount でパッケージ内のファイルを上書きしている (パッケージ本体は無改変)。
- `scripts/rgbd_republisher.py`: carrobo のファイル名・ノード名 (`rgbd_republisher`) をそのまま採用 (旧 `openmm_rgbd_compression_republisher.py` から改名)。`Dockerfile.ros2` の ADD 先も変えたので ros2 イメージは要再ビルド (compose の bind mount があるので再ビルド前でも起動は通る)。

## 10. 設定・実装上の判断で、後で見直すかもしれないもの

- ステップ4 の分割 (2026-10-12): `hsr.py` 4,114 行 → `hsr.py` 1,230 行 + 8 モジュール (mixin 方式。`class hsr(SensorsMixin,
  PhysicsTuningMixin, GraspMixin, BaseDriveMixin)`)。メソッドの移動のみで中身は無変更。`launch_isaacsim.py` からは
  `task_config.py` と `drawers.py` を切り出した (残りの spawn / reset / サービスはモジュール変数を共有しているので据え置き)。
  compose は単一ファイル mount なので、新モジュールを増やしたら `env_docker/docker-compose.yml` と `Dockerfile.isaacsim` の両方に足すこと
  (dev モードは scripts/ ごと mount するので不要)。

- `configs/dressing.yaml` の `rc26_venue` preset に `wall_color` を追加 (tidyup の `/wall_*` を白くするため)。
  他タスクの world に `/wall_*` prim は無いので見た目は変わらないはずだが、未確認。
- `launch_isaacsim.py` の床: 100 m の厚い立方体 → world 範囲 + `FLOOR_MARGIN=1.5` の 5 cm 板 (carrobo 方式)。
  dressing の room_size も同じ margin (旧 0.5) を使うので、rc26.world / restaurant では床テクスチャが片側 1 m 広がる。
- `object_placement.py` は `OBJECT_PLACEMENT_CONFIG` / `PLACEMENT_CONFIG` 環境変数も後方互換で読む。
  `SEED=` 経由の `PLACEMENT_SEED` に一本化するなら削る。
- `configs/tasks/tidyup/object_list.yaml` (物体一覧と共通座標) はコードから読まれない参考ファイル。
- `GRASP_ATTACH` 既定 0、`ROS_DOMAIN_ID` 既定 26、MoveIt 上書き mount: carrobo の値をそのまま採用 (決定済み)。
- `recordings/.gitkeep` だけ追跡し中身は無視 (`.gitignore`)。

## 11. まだ作っていないもの

- GitHub の `ry0hei-kobayashi/hsr-omniverse2` リポジトリ (remote `origin` の URL だけ設定済み)。
- hsr-omniverse2 の初回 commit (案は `~/hsr-omniverse2-patches/COMMIT_PLAN.md`)。
- `make up pc` (別 PC モード) の 5.1 での動作確認。
- `configs/tasks/{gpsr,hri,laundry,pick_and_place,restaurant}` での `arena_cameras: true` / `make tune` の動作確認
  (カメラ位置は床中心から自動配置されるはずだが未確認)。
