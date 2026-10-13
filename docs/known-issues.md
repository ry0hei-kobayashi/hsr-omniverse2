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
- `env_docker/Dockerfile.ycb-grasp` / `docker-compose-ycb-grasp.yml` は 2026-10-12 に Jazzy 化した: 旧版 (Isaac 2023.1.1 + apt Humble)
  を捨て、本体イメージ `hsr-omniverse-isaacsim:jazzy` の上に pyassimp と tmc_wrs_gazebo (jazzy) の STL だけ足す。`make ycb-grasp [localhost]
  [YCB_MAX_MODELS=n]` で実行 (ros2 コンテナ不要)。`sample-ycb-grasp.py` は `usd/wrs_models` の場所 (旧コードは `scripts/usd/...` を見ていた)、
  STL の場所 (`WRS_MODELS_DIR`)、assets root (`get_version()[0]` → launch_isaacsim.py と同じ決め方) を直した。
  動作確認 (2 物体): 1 個目は把持まで通る。2 個目で `'Articulation' object has no attribute '_physics_view'` が出て把持できない
  (`[base-brake]` も同じエラー)。1 個目の `DeletePrims` で physics scene が再構築され、`hsr.py:154` の `Articulation` view が無効になる
  ため。サンプルは例外を握って次へ進むので完走はするが、2 個目以降は実質失敗。対処候補: 物体削除後に `robot.robots.initialize()` を
  呼び直す、または削除せず非表示にする。未対応。
- 検証: `make up localhost TASK=gpsr` で起動し、FJT 移動・`/gripper_controller/grasp` (tmc_control_msgs)・IMU・wheel_odom・カメラ・scan を確認。Python エラーなし。

## 9. 外部由来コードの出所・ライセンス

- `3rdparty/semu.robotics.ros2_bridge` (submodule): Toni-SM / MIT。fork ブランチ `isaac5.1-jazzy` (Humble 用は `isaac5.1-humble`) は上流と同じ拡張構成のまま
  Python 3.11 / Isaac 5.1 向けに作り直したもの (`tools/build.sh`: control_msgs + add_on_msgs の py3.11 ビルド、
  Cython 版 `_ros2_bridge.cpython-311-*.so`、`exts/` パッケージ)。上流と同じく成果物 (約 11 MB) をコミットする方針。
  `add_on_msgs` の .srv は上流 README から復元 (`src/add_on_msgs`)。ヘッドレス Isaac 5.1 で拡張をロードし
  `/get_prim_attribute` の get/set が通ることを確認済み。hsr-omniverse2 自体は拡張ではなく `ros2_bridge.py` をソース import する
  (速度・安定性の判断。`docs/known-issues.md` 作成時点)。
  fork ブランチは push 済み (`isaac5.1-humble` = `692def7`、`isaac5.1-jazzy` = `d12bbfc`)。hsr-omniverse2 の gitlink は
  `d12bbfc`、`.gitmodules` は `branch = isaac5.1-jazzy`。
  `tools/build.sh` の注意: Isaac イメージは `ACCEPT_EULA=Y` と `--entrypoint bash` が必要 (既に script に組み込み済み)。
- msgbuild ステージで clone するもの: `ros-controls/control_msgs` (jazzy, BSD-3)、`hsr-project/tmc_realtime_control` (jazzy)、
  `ry0hei-kobayashi/tmc_wrs_gazebo` の `tmc_wrs_gazebo_worlds` (Python 部分のみコピー。`docs/tmc_wrs_gazebo_worlds-build-note.md`)。
  LICENSE はイメージ内 `/app/ros_msgs/licenses/` に同梱。ブランチ名での clone なので再現性は低い (commit で固定したければ Dockerfile に SHA を書く)。
- `assets/joint_limits-smooth.yaml` / `assets/moveit-no-octomap.rviz`: `hsrb_moveit_config` (hsr-project) の設定を改変したもの。
  bind mount でパッケージ内のファイルを上書きしている (パッケージ本体は無改変)。
- `scripts/rgbd_republisher.py`: carrobo のファイル名・ノード名 (`rgbd_republisher`) をそのまま採用 (旧 `openmm_rgbd_compression_republisher.py` から改名)。`Dockerfile.ros2` の ADD 先も変えたので ros2 イメージは要再ビルド (compose の bind mount があるので再ビルド前でも起動は通る)。

## 10. 設定・実装上の判断で、後で見直すかもしれないもの

- ステップ4 の分割 (2026-10-12): `hsr.py` 4,114 行 → `hsr.py` 1,230 行 + 8 モジュール (mixin 方式。`class hsr(SensorsMixin,
  PhysicsTuningMixin, GraspMixin, BaseDriveMixin)`)。メソッドの移動のみで中身は無変更。`launch_isaacsim.py` からは
  `task_config.py` と `drawers.py` を切り出した (残りの spawn / reset / サービスはモジュール変数を共有しているので据え置き)。
  (2026-10-12 の再編で compose / Dockerfile は `sim/` ごと (§10d) になったので、パッケージ内にファイルを足しても追記不要。)

- `configs/dressing.yaml` の `rc26_venue` preset に `wall_color` を追加 (tidyup の `/wall_*` を白くするため)。
  他タスクの world に `/wall_*` prim は無いので見た目は変わらないはずだが、未確認。
- `launch_isaacsim.py` の床: 100 m の厚い立方体 → world 範囲 + `FLOOR_MARGIN=1.5` の 5 cm 板 (carrobo 方式)。
  dressing の room_size も同じ margin (旧 0.5) を使うので、rc26.world / restaurant では床テクスチャが片側 1 m 広がる。
- `object_placement.py` は `OBJECT_PLACEMENT_CONFIG` / `PLACEMENT_CONFIG` 環境変数も後方互換で読む。
  `SEED=` 経由の `PLACEMENT_SEED` に一本化するなら削る。
- `configs/tasks/tidyup/object_list.yaml` (物体一覧と共通座標) はコードから読まれない参考ファイル。
- `GRASP_ATTACH` 既定 0、`ROS_DOMAIN_ID` 既定 26、MoveIt 上書き mount: carrobo の値をそのまま採用 (決定済み)。
- `recordings/.gitkeep` だけ追跡し中身は無視 (`.gitignore`)。

## 10b. scripts/ の再編 (2026-10-12)

- 平置き 28 ファイルを役割別のパッケージに分けた (ユーザー決定: 6 ディレクトリ、`robot/` の下に hsr と hsrc_ex、root の
  `scene_dressing/` は `scripts/env/` へ):
  `robot/hsr/` (robot.py=旧 hsr.py, params, quat=旧 hsr_math, kinematics, action_servers, sensors, physics, grasp, base_drive)、
  `robot/hsrc_ex/robot.py` (旧 hsr_hsrc_ex.py)、`env/` (construct_environment, dressing_presets, furniture_spawn, people_spawn,
  object_placement, drawers, scene_dressing/)、`task/` (task_config, arena_cameras, apply_tune)、`tools/` (gen_cyclonedds_pc.sh,
  isaac_localhost_mode.sh, convert_object_to_ycb_physics,
  fetch_isaac_offline_assets, list_people_anims)、`samples/`。`launch_isaacsim.py` と新設 `repo_paths.py` は scripts/ 直下。
- import は絶対パッケージ名 (`from robot.hsr import robot as hsr`、`from env import object_placement`、`from task import task_config`)。
  コンテナ内は /app/robot, /app/env, ... (PYTHONPATH に /app)。dev モードは /app/scripts が sys.path[0] なのでホスト側が優先。
- リポジトリルートの解決は `repo_paths.REPO_ROOT` (configs/ と usd/ を両方持つ最初の親ディレクトリ、無ければ /app) に統一。
  旧コードの `dirname(dirname(__file__))` は深さが変わると壊れるため全て置き換えた。`task/apply_tune.py` はホスト専用なので
  3 段 dirname のまま。シェル 2 本 (`tools/`) は `../..`。
- 注意: `scripts/tools/isaac_localhost_mode.sh` のパスが変わった。Apptainer 側の手順 (`docs/dds-localhost.md`) も更新済み。
  外部 (hma2_ws 等) に旧パスを書いている場合は直す必要がある。
- 検証 (2026-10-12、`make up localhost TASK=gpsr`、再ビルド後): 起動ログに Traceback 無し、topic 109、`/isaac/get_entities` /
  `/isaac/get_entity_state` PASS、head の FollowJointTrajectory 往復 SUCCEEDED (pan 0→0.487→0)。dev 配置 (/app/scripts mount) でも
  `repo_paths.REPO_ROOT=/app` と `env` / `task` / `robot.hsr` の純 Python 部分の import を確認。ycb-grasp イメージも再ビルド済み
  (実走は再編前に 2 物体で確認)。`.dockerignore` を追加し、ホストの `__pycache__` がイメージに入らないようにした。
- 未検証: hsrc_ex (`ROBOT=hsrc_ex`、§7 のとおり対象外)、`make dev up` の実走 (`.env` が無い)、`make tune` / 競技モードの再編後の実走。
- 追記 (同日): `scripts/ros2/` は §10c の ROS 2 パッケージへ移したので、`scripts/` は Isaac 側の Python だけになった。

## 10c. ros2 側を ROS 2 パッケージ `hsr_omniverse_ros2` にまとめた (2026-10-12)

- ユーザー提案 (「hsr-omniverse2 の中にメタ ROS パッケージを作る」) を受け、ros2 コンテナ側に散らばっていたものを
  `hsr_omniverse_ros2/` (ament_python) に集めた: `launch/` (hsr.launch.py, teleop.launch.py, hsrb_relay_topics.launch.xml,
  hsrb_sensor_frames.launch.xml、旧 repo 直下 `launch/`)、`hsr_omniverse_ros2/` (rgbd_republisher, reset_world_matcher_helper、
  旧 `scripts/ros2/`。console_scripts で `ros2 run hsr_omniverse_ros2 <name>`)。
  HSR 依存 (hsrb_*, tmc_*, control_msgs, tmc_control_msgs, simulation_interfaces …) は `package.xml` の exec_depend に集約した
  (方針: HSR 依存ライブラリは ros2 docker 側に持つ。Isaac 側には置かない)。純粋なメタパッケージ (package.xml のみ) ではなく
  launch とノードを持つ普通のパッケージにしたのは、置き場所が無かった中継ノード (将来の GripperApplyEffort 中継を含む) を
  収める先を兼ねるため。パッケージ名にハイフンは使えないので `hsr_omniverse_ros2`。
- 変更点: `Dockerfile.ros2` は 127 パッケージの層の後ろに `ADD hsr_omniverse_ros2 /ws/src/hsr_omniverse_ros2` +
  `rosdep install` + `colcon build --symlink-install --packages-select hsr_omniverse_ros2` の層を追加 (既存層は無効化されない)。
  `CMD` と compose の `command` は `ros2 launch hsr_omniverse_ros2 hsr.launch.py`。compose は旧来のファイル単位 mount
  (`/hsr.launch.py`, `/rgbd_republisher.py`) をやめ、`../hsr_omniverse_ros2:/ws/src/hsr_omniverse_ros2:ro` のディレクトリ mount に
  した (symlink-install なので launch / ノードの編集は再ビルド無しで反映。package.xml / setup.py を変えたときは `make build`)。
  hsr.launch.py 内の相対パス (XML include、teleop.launch.py、スクリプトの絶対パス fallback) は
  `get_package_share_directory('hsr_omniverse_ros2')` と `Node(package='hsr_omniverse_ros2', executable=...)` に置き換えた。
  `/examples/grasp_tf_broadcaster.py` (教材、mount) と `/hsr-omniverse.rviz`、`/map.yaml` はそのまま。
- hma2_ws (Apptainer) 側でも同じ launch / 中継ノードを使いたい場合は、`hsr_omniverse_ros2` を vcs import か submodule で
  ワークスペースに入れれば `ros2 launch hsr_omniverse_ros2 ...` できる (未実施)。
- hsr-project のソース取得 (git clone ループ) は package.xml では置き換えられない (`.repos` + vcs の領域)。今回は触っていない。
- 検証 (2026-10-12、`make up localhost TASK=gpsr`、ros2 再ビルド後): PID 1 が `ros2 launch hsr_omniverse_ros2 hsr.launch.py`、
  `/ws/src/hsr_omniverse_ros2` がホストの bind mount で、install 側の launch symlink がそこへ解決。rgbd_republisher /
  reset_world_matcher_helper は `/ws/install/hsr_omniverse_ros2/lib/hsr_omniverse_ros2/` の console_scripts として起動。
  node 61・topic 109 (再編前と同数)、compressed / compressedDepth / rectified_points あり、teleop ノードあり、
  head の FollowJointTrajectory 往復 SUCCEEDED、ログに Traceback / process has died 無し。
- rosdep の新しい層は `--from-paths src --ignore-src -r` (上の層と同じ)。hsr-project 側の未解決キー 5 件 (tmc_pgr_camera,
  yolox_ros_cpp, numpy, exxx_control_table, bboxes_ex_msgs) は以前からのもので `-r` で無視している。hsr_omniverse_ros2 自身の
  キーは全て解決した。
- 未実施: Apptainer (hma2_ws) 側への取り込み、`.repos` 化、`assets/hsr-omniverse.rviz` と `/map.yaml` のパッケージ share への移動。

## 10d. Isaac 側も Python パッケージ `hsr_omniverse_sim` にした (`scripts/` → `sim/`、2026-10-12)

- ユーザー指摘: §10b で足した `repo_paths.py` (configs/ と usd/ を探して親を遡る) はパッケージ参照になっていない。
  ros2 側 (§10c) と同じ考え方で、Isaac 側のコードも pip パッケージにして Isaac の Python (3.11) に editable install する。
- 配置: `pyproject.toml` + `hsr_omniverse_sim/` (launch_isaacsim.py, paths.py, robot/ env/ task/ samples/ tools/)。
  ホスト側の補助 (`apply_tune.py`, `gen_cyclonedds_pc.sh`, `isaac_localhost_mode.sh`) はパッケージに入れず `tools/`
  (リポジトリルートは従来どおり `../..` / 3 段 dirname)。import は `from hsr_omniverse_sim.robot.hsr import robot` のように
  パッケージ名付きの絶対 import。`repo_paths.py` は削除し、`paths.REPO_ROOT = Path(__file__).resolve().parents[2]`
  (固定深さ 1 行。環境変数 `HSR_OMNIVERSE_ROOT` で上書き可) に置き換えた。usd/ configs/ worlds/ はパッケージの外に
  残るので「ルート」の概念自体は残る (巨大・submodule のためパッケージに同梱しない)。
- `Dockerfile.isaacsim`: 個別 ADD 7 行 → `ADD sim /app/sim` + `pip install --no-deps --no-build-isolation -e /app/sim`
  (Isaac 同梱の setuptools 78 で建つ)。`PYTHONPATH` から `/app` を外した (ros_msgs と同梱 rclpy だけ)。
  CMD / Makefile の `LAUNCH_PY` は `-m hsr_omniverse_sim.launch_isaacsim` (dev と通常で同一になった)。
  compose は `../hsr_omniverse_sim:/app/hsr_omniverse_sim:ro` の 1 行 (editable なので編集が再ビルド無しで反映。`pyproject.toml` を変えたときだけ
  `make build`)。`docker-compose.dev.yml` の `scripts` mount は削除。ycb-grasp も `/app/sim` を使う。
- 残り: semu の `sys.path.insert` (`robot/hsr/action_servers.py`) は fork に `pyproject.toml` を足して同じく pip install
  すれば消せる (fork 変更なので別ステップ)。
- 検証 (2026-10-12、Isaac 再ビルド後 `make up localhost TASK=gpsr`): イメージ内で `pip show` が
  `Editable project location: /app/sim`、`REPO_ROOT=/app`、`HSR_OMNIVERSE_ROOT` 上書き可、PYTHONPATH に `/app` 無し。
  起動時の PID 1 は `python.sh -m hsr_omniverse_sim.launch_isaacsim`、`/app/sim` はホストの bind mount。
  node 61 / topic 109、`/isaac/get_entities` 431 件 (RESULT_OK)、`/isaac/get_entity_state hsrb` OK、head の
  FollowJointTrajectory 往復 SUCCEEDED、ログに Traceback / ImportError 無し。ycb-grasp イメージも再ビルドし
  `/app/hsr_omniverse_sim/samples/` と `REPO_ROOT=/app` を確認 (実走は未)。`make -n run dev` / `list-anims` /
  `tune-apply` の展開も確認。

## 10e. データを `io/` にまとめ、`examples/` を削除 (2026-10-12)

- ユーザー指示 (「io/configs io/models/usd のようにまとめられるものはまとめる、examples は消す」)。
  `configs/` → `io/configs/`、`usd/` → `io/models/usd/` (submodule `hsrb` / `wrs_models` も `git mv`。`.gitmodules` の
  path は更新済み、name は旧名 `usd/hsrb` のまま)、`worlds/` → `io/models/worlds/`、`recordings/` → `io/recordings/`。
  `assets/` (DDS xml / entrypoint / rviz / MoveIt 上書き) はユーザー指示で今回は据え置き。`task_starter/` も未変更。
- コード側はコンテナ内パスの直書き (`/app/usd/...`, `/app/configs/...`, `/app/worlds/...`) と `REPO_ROOT` 相対の二重候補を
  やめ、`paths.py` の `CONFIGS_DIR` / `USD_DIR` / `WORLDS_DIR` (`REPO_ROOT/io/...`) 1 本にした。コンテナは `../io:/app/io:ro`
  の 1 mount (+ `io/configs/textures` → `/data/textures`、`io/recordings` → `/recordings` rw は従来どおり)。
  `Dockerfile.isaacsim` は `ADD io /app/io`。`.dockerignore` で `io/recordings/` は除外。
- `examples/` (hsrb_interface / MoveIt のサンプル 9 本) を削除。それに伴い `hsr.launch.py` の `use_grasp_tf`
  (`/examples/grasp_tf_broadcaster.py` を起動していた) も削除。compose の `/examples` mount も削除。
- 注意: 外部 (hma2_ws 等) から `configs/` `usd/` のパスを直接参照していた場合は `io/` 付きに直す必要がある。
  `io/recordings/` に旧 `recordings/` の録画フォルダがそのまま残っている (git 管理外)。
- 検証 (2026-10-12、両イメージ再ビルド後 `make up localhost TASK=gpsr`): イメージ内 `/app/io/{configs,models/usd,models/worlds}`、
  `paths.CONFIGS_DIR=/app/io/configs` 等、起動時の mount は `io` → `/app/io` (ro)、`io/configs/textures` → `/data/textures`、
  `io/recordings` → `/recordings` (rw)。ログ: task dir / placement / world / isaac_offline ミラー / people が全て `/app/io/...` で解決、
  物体 spawn OK、Traceback 無し。node 60 / topic 108 (grasp_tf_broadcaster 分だけ減少)、`/isaac/get_entities` 431 件、
  `/isaac/get_entity_state hsrb` OK、head の FollowJointTrajectory 往復 SUCCEEDED。

## 10f. `sim/` `ros2/` の階層をやめ、パッケージをリポジトリ直下に (2026-10-12)

- ユーザー指摘 (「sim / ros2 の階層はいらない。hsr_omniverse_ros2 と hsr_omniverse_sim を親にする」)。名前は「2」無しに決定。
  `sim/hsr_omniverse2_sim/` → `hsr_omniverse_sim/` (Python パッケージ、import は `hsr_omniverse_sim.*`)、
  `sim/pyproject.toml` → `pyproject.toml` (リポジトリ直下、`name = hsr-omniverse-sim`)、`sim/tools/` → `tools/`、
  `ros2/hsr_omniverse2/` → `hsr_omniverse_ros2/` (ROS パッケージ名も `hsr_omniverse_ros2`。ament_python の慣例で中に同名の
  モジュール dir `hsr_omniverse_ros2/hsr_omniverse_ros2/` は残る)。`sim/README.md` は README の構成表に統合して削除。
- 深さ依存の 4 箇所を直した: `paths.REPO_ROOT` は `parents[1]`、`tools/apply_tune.py` は 2 段 dirname、
  `tools/gen_cyclonedds_pc.sh` と `tools/isaac_localhost_mode.sh` は `..`。
- コンテナ: `Dockerfile.isaacsim` は `ADD pyproject.toml` + `ADD hsr_omniverse_sim` + `pip install -e /app`
。compose は `../hsr_omniverse_sim:/app/hsr_omniverse_sim:ro`。
  ros2 側は `/ws/src/hsr_omniverse_ros2`、起動は `ros2 launch hsr_omniverse_ros2 hsr.launch.py`。
- §10c / §10d の記述は当時のパス (`ros2/hsr_omniverse2`, `sim/`, `/app/sim`) のまま残している。
- 外部 (hma2_ws 等) の `source .../sim/tools/isaac_localhost_mode.sh` は `tools/isaac_localhost_mode.sh` に直す必要がある。
- 検証 (2026-10-12、両イメージ再ビルド後 `make up localhost TASK=gpsr`): `pip show hsr-omniverse-sim` が
  `Editable project location: /app`、`REPO_ROOT=/app`。PID 1 は `python.sh -m hsr_omniverse_sim.launch_isaacsim` と
  `ros2 launch hsr_omniverse_ros2 hsr.launch.py`、launch の symlink は bind mount の `/ws/src/hsr_omniverse_ros2/launch/` に解決。
  node 60 / topic 108、`/isaac/get_entities` 431 件、`/isaac/get_entity_state hsrb` OK、head の FollowJointTrajectory
  往復 SUCCEEDED、Traceback / process has died 無し。

## 10g. `tools/` と `assets/` を分類し直した (2026-10-12)

- ユーザー指示 (「tools と assets を分類して配置し直す」。DDS 用の dds/ は作らず assets/ に残す)。
  - `assets/`: DDS 関連だけに絞った。`cyclonedds{,.localhost,.pc}.xml` に加え、`gen_cyclonedds_pc.sh` (pc.xml の生成) と
    `isaac_localhost_mode.sh` (Apptainer 側の切替。プロファイルと同じディレクトリを見るので `..` の計算が不要になった) を移動。
  - `env_docker/`: `ros_entrypoint.isaacsim.sh` / `ros_entrypoint.ros2.sh` (Dockerfile 専用) を移動。
  - `hsr_omniverse_ros2/config/`: `hsr-omniverse.rviz`、`moveit-no-octomap.rviz`、`joint_limits-smooth.yaml` (ros2 側の設定)。
    Dockerfile.ros2 の ADD と compose の MoveIt 上書き mount の元パスを変更 (コンテナ内の配置は従来どおり)。
  - `hsr_omniverse_sim/tools/apply_tune.py`: `make tune-apply` はホストの python3 で `python3 -m hsr_omniverse_sim.tools.apply_tune`
    (リポジトリ直下で実行。`paths.CONFIGS_DIR` を使う。Isaac 依存なし)。`tools/` は削除。
- 外部 (hma2_ws 等) の `source` 先は `assets/isaac_localhost_mode.sh` になる (`docs/dds-localhost.md` 更新済み)。
- 検証 (2026-10-12): `source assets/isaac_localhost_mode.sh 31` で CYCLONEDDS_URI が assets/ のプロファイルを指す、
  `make -n up pc PEER=…` が `./assets/gen_cyclonedds_pc.sh` を呼ぶ、`python3 -m hsr_omniverse_sim.tools.apply_tune` の
  TUNED / 書き込み先が `io/recordings/tune/…` / `io/configs/tasks/tidyup/placement.yaml` に解決。両イメージ再ビルド後
  `make up localhost TASK=gpsr`: `/ros_entrypoint.sh` と `/hsr-omniverse.rviz` が新パスから ADD され、MoveIt 上書き mount は
  `hsr_omniverse_ros2/config/` から (moveit.rviz に octomap 無し)。get_entities 431 件、head FJT 往復 SUCCEEDED、Traceback 無し。

## 10h. `io/` をやめ、config と worlds はパッケージ内、USD は `models/` に (2026-10-12)

- ユーザー指摘「paths が結局残っている」→ 方針: データをパッケージ内に統合し `config` と `data` に分ける。外部由来の USD
  (submodule 等) をパッケージに入れるのは問題なので、USD は全部パッケージ外 (ユーザー決定)。
  - `hsr_omniverse_sim/config/` (旧 io/configs: placement.yaml, dressing.yaml, tasks/, textures/)、
    `hsr_omniverse_sim/data/worlds/` (旧 io/models/worlds)。コードからは `importlib.resources.files('hsr_omniverse_sim') / 'config'`
    で参照 (editable install なので実ファイルのパスが返る)。`__file__` の親を数える計算は無くなり、`paths.py` は削除。
  - `models/` (旧 io/models/usd、1.5 GB。hsrb / wrs_models は submodule、isaac_offline は NVIDIA のミラー、rc26* 等は自作)。
    パッケージ外なので置き場所は環境変数 `HSR_MODELS_DIR` 1 本で渡す (`Dockerfile.isaacsim` の `ENV HSR_MODELS_DIR=/app/models`、
    compose が `../models:/app/models:ro`)。利用側は `os.environ['HSR_MODELS_DIR']` (未設定なら KeyError で止まる)。
  - `recordings/` はリポジトリ直下に戻した (出力をパッケージ内に書かないため)。
  - semu の `sys.path` 挿入 (`robot/hsr/action_servers.py`) は `/app/3rdparty/...` 固定 (`SEMU_ROS2_BRIDGE_SRC` で上書き可)。
    fork に pyproject を足して pip install すれば消せる (未実施)。
  - `apply_tune.py` はカレント (リポジトリ直下 = `make tune-apply`) の `recordings/tune/` を読む。
- `/data/textures` (dressing.yaml がコンテナ内パスで参照) は従来どおり `hsr_omniverse_sim/config/textures` を ADD / mount している。
  パッケージ内相対に変えれば mount も不要になる (未実施)。
- pyproject は editable 専用。非 editable (`pip install .`) にすると config/ (7 MB) もコピーされるが USD は入らない。
- §10e〜§10g の記述は当時のパス (`io/...`) のまま残している。
- 検証 (2026-10-12): 両イメージ再ビルド OK。イメージ内で `HSR_MODELS_DIR=/app/models`、`/app/models` に USD 11 ディレクトリ、
  `files('hsr_omniverse_sim') / 'config'` が `/app/hsr_omniverse_sim/config` (PosixPath) に解決、`task_config.CONFIG_DIR` も同じ。
  ホストで `python3 -m hsr_omniverse_sim.tools.apply_tune` の CONFIG_DIR / TUNED / 書き込み先が解決。compose の mount は
  `hsr_omniverse_sim` / `models` / `recordings` / `config/textures` の 4 つ。`make up localhost TASK=gpsr` の起動ログでは
  task dir / placement / world / isaac_offline ミラー / people / furniture が全て `/app/hsr_omniverse_sim/...` と `/app/models/...`
  で解決し、ロボット spawn まで到達した。**ただし ROS 側の最終確認 (get_entities / FJT) は未完**: 同じ GPU で別プロセス
  (`jsdn-handover-baselines` の python、43 GB) が動いていて Isaac が `ERROR_OUT_OF_DEVICE_MEMORY` で落ちた (2 回)。
  VRAM が空いたら `make up localhost TASK=gpsr` → head の FollowJointTrajectory と `/isaac/get_entities` を再確認すること。

## 11. まだ作っていないもの

- GitHub の `ry0hei-kobayashi/hsr-omniverse2` リポジトリ (remote `origin` の URL だけ設定済み)。
- hsr-omniverse2 の初回 commit (案は `~/hsr-omniverse2-patches/COMMIT_PLAN.md`)。
- `make up pc` (別 PC モード) の 5.1 での動作確認。
- `configs/tasks/{gpsr,hri,laundry,pick_and_place,restaurant}` での `arena_cameras: true` / `make tune` の動作確認
  (カメラ位置は床中心から自動配置されるはずだが未確認)。
- 1 台の PC で複数インスタンス (`make up N=2`)。設計と衝突箇所の調査は `docs/multi-instance-design.md` (2026-10-12、未実装)。
  現状は compose project 名・ROS_DOMAIN_ID・squid の 3128・Kit の data/logs が固定で 2 つ目が 1 つ目を乗っ取る。
  `pc` モードで `ROS_DOMAIN_ID=49 make up pc` が効かない (`Makefile:101` の `:=` 上書き) のも同じ文書に記載。

## 12. Jazzy 化 (ステップ5、2026-10-12)

- 構成: **Jazzy 専用** (2026-10-12 決定。当初は `ROS_DISTRO` build arg で Humble / Jazzy を切り替える形にしたが、
  ブランチ `ry0hei/devel/jazzy` では Humble 分岐を削除した。Humble 構成が要るなら `main` (ステップ4 まで) を使う)。
  Isaac 側は同梱の `isaacsim.ros2.bridge/jazzy/rclpy` (Python 3.11、87 パッケージ) を使い、同梱に無いメッセージは
  `ros:jazzy-ros-base` (Python 3.12) に python3.11 を足した msgbuild ステージで建てる。`ros2` 側は
  https://github.com/ry0hei-kobayashi/hsr-jazzy-docker と同じ hsr-project の `jazzy` ブランチ群。
- Humble 構成 (main) との差 (ros2 コンテナの clone 対象): `Hibikino-Musashi-Home/hsrb_common` (rc26 fork、humble のみ) の代わりに
  `hsr-project/hsrb_common` jazzy を使う (rc26 fork 固有の変更があるなら未反映)。`tmc_manipulation_local_planner` と
  `hsrb_robot_local_planner` は jazzy ブランチが無いので入れない (launch からは未参照)。
  `tmc_wrs_gazebo` は ry0hei-kobayashi fork の `jazzy` ブランチ。
- semu fork は Jazzy 用ブランチ `isaac5.1-jazzy` (1.1.0) を作り、`bin/` / `packages/` / `exts/` を Jazzy の control_msgs で
  建て直した (`tools/build.sh jazzy`)。hsr-omniverse2 の `.gitmodules` はこのブランチを指す。Humble 用は `isaac5.1-humble`
  (692def7) のまま残してある。hsr-omniverse2 自体は fork の `packages/` を sys.path に入れず `/app/ros_msgs` を使う
  (ソース import のみ)。
- Jazzy で踏んだ差分 (直し済み、fork 側): `control_msgs/JointTrajectoryControllerState` から旧欄 `actual` / `desired` が
  消えた (control_msgs >= 4.6)。semu fork の controller_state publisher が毎ステップ AttributeError を出していたので、
  旧欄は `hasattr` で有る時だけ書く形にした (fork 1.0.1、成果物も再ビルド済み。**fork の commit / push と hsr-omniverse2 の
  gitlink 更新が必要**)。Humble 側の購読者 (pumas の arm_node 等) が `actual` を読んでいる場合、Jazzy では `feedback` を読む
  必要がある。
- `ROS_LOCALHOST_ONLY` は Jazzy で非推奨 (`ROS_AUTOMATIC_DISCOVERY_RANGE` に置き換え)。このリポジトリの DDS 設定は
  CycloneDDS の XML で閉じているので影響しないが、`scripts/isaac_localhost_mode.sh` の `export ROS_LOCALHOST_ONLY=0`
  は Jazzy では警告が出る可能性がある。
- メッセージの 3.11 ビルドが 2 箇所 (fork の `packages/` と hsr-omniverse2 の msgbuild ステージ) にあり `control_msgs` が
  重複している。改善案は「ビルド済みを `ros_msgs/` にコミットして Dockerfile を単段にする」(fork と同じ方式) か
  「semu を Kit 拡張としてロードして fork の `packages/` を使う」。未着手 (2026-10-12 時点)。
- Isaac 側で 3.11 向けに建てる対象を減らした (2026-10-12):
  - `gazebo_msgs` を廃止。旧 `/gazebo/get_world_properties` / `/gazebo/get_model_state` は、Isaac 同梱かつ apt Jazzy にもある
    ROS 2 標準の `simulation_interfaces` に置き換えた: `/isaac/get_entities` (GetEntities、`filters.filter` は正規表現) と
    `/isaac/get_entity_state` (GetEntityState、**world 基準**の姿勢。旧版の `relative_entity_name` 相対は廃止)。
    Isaac 同梱は 1.1.0、ros2 コンテナの apt は 1.5.1 だが、この 2 サービスの定義は同一であることを確認済み。
    旧サービスの利用者はこのリポジトリ内に無い (tmc_gazebo_task_evaluators 用だったが Jazzy ワークスペースに無い)。
  - `tf_transformations` を廃止。使っていたのは `quaternion_from_euler` / `euler_from_quaternion` の 2 関数だけなので
    `scripts/hsr_math.py` に純 Python で実装 (sxyz、(x,y,z,w))。ros2 コンテナの tf_transformations と 20,000 点で比較し、
    pitch=±90° から 1e-9 以上離れていれば差 < 1e-12。特異点 1e-9 以内では分解が ill-conditioned で最大 2.5e-4 rad 違う
    (どちらも同じ回転の近似。使い所は台車 yaw なので影響なし)。pip の transforms3d も不要になり削除。
  - 残る 3.11 ビルド対象は `control_msgs` と `tmc_control_msgs` の 2 つ。`control_msgs` は semu fork の `packages/` にも
    3.11 版がある。`tmc_control_msgs` は GripperApplyEffort の action サーバが Isaac 側 (`hsr_action_servers.py`) にあるため
    Isaac が import している。Isaac 側を semu のみにするなら、このサーバを ros2 コンテナの中継ノード (GripperApplyEffort →
    semu の GripperCommand) に移す必要がある (未決定)。
  - 検証 (2026-10-12、`make up localhost TASK=gpsr`): `/isaac/get_entities` 431 件・正規表現フィルタ・`/isaac/get_entity_state`
    (hsrb、frame=world)・未知名で RESULT_NOT_FOUND、cmd_vel で 4 秒回転して `/omni_base_controller/wheel_odom` 187 msg、
    yaw 2.16 rad、|q|=1.0000 (hsr_math の純 Python 実装経由)。起動ログに Traceback 無し。
- 参考: arimura 版 (`~/hsr-omniverse` の `arimura/devel/isaac4.5.0`) ではこの問題は存在しなかった。Isaac Sim 4.5.0 は Ubuntu 22.04 上で
  **Python 3.10.15** を同梱しており、これは jammy の system Python と同じ版なので、Isaac イメージに apt の `ros-humble-*`
  (rclpy / control_msgs / gazebo_msgs / tf_transformations …) をそのまま入れ、`tmc_realtime_control` を colcon で建てて
  `source /opt/ros/humble/setup.bash` するだけで Isaac の Python から import できた (`Dockerfile.isaacsim-ros2`、
  `assets/ros_entrypoint.sh.ros2`)。Isaac 5.1 は Ubuntu 24.04 上で Python 3.11 を同梱し、noble の Jazzy は 3.12 なので
  この方式が成立しなくなった。これが msgbuild ステージ (3.11 向け再ビルド) が必要になった直接の理由。
- msgbuild の再現性 (2026-10-12 に踏んだ): `control_msgs` を `-b jazzy` (ブランチ HEAD) で clone していたため、上流に
  `ExecuteMotionPrimitiveSequence.idl` が追加された時点で venv の旧 `lark-parser` 0.12 が `UnboundLocalError (iter_subtrees の queue)`
  で落ち、ビルド不能になった。対処: venv の lark を apt Jazzy と同じ `lark==1.1.9` にし、`control_msgs` は ros2 コンテナの apt 版
  (`ros-jazzy-control-msgs` 5.10.0) と同じタグ `5.10.0` に固定 (型ハッシュも揃う。`ARG CONTROL_MSGS_VERSION`)。
  **semu fork の `tools/Dockerfile.msgbuild` も同じ `lark-parser` + `-b jazzy` なので、次に `tools/build.sh` を回すと同じ失敗をする。**
  fork 側も `lark==1.1.9` と `5.10.0` 固定に直す必要がある (fork の変更・commit はユーザー)。
  補足 (同日に確認): 失敗箇所は IDL 固有ではなく lark-parser 0.12 の文法コンパイル (`Lark(grammar)` → `iter_subtrees`) で、
  python3.11 (deadsnakes 3.11.17) との組み合わせで出る。fork の d12bbfc は同じレシピで同日の数時間前に通っており、
  その間に deadsnakes の python3.11 が上がった可能性が高い (未確定)。fork の成果物は `jazzy` HEAD (2eaaa54) 由来で、
  apt の 5.10.0 との差は `ExecuteMotionPrimitiveSequence` の定数 2 つ (2026-10-03 追加) だけ。FollowJointTrajectory /
  GripperCommand の定義は同一なので実害は無い。
- **msg の残課題は保留 (2026-10-12、ユーザー判断: 現状で動くので一旦保留)。** 保留した選択肢:
  1. `control_msgs` を msgbuild ではなく semu fork の `packages/` + `bin/` から使う (上記 fork 修正が前提)。
  2. GripperApplyEffort の action サーバを ros2 コンテナの中継ノードに移し、Isaac 側は semu の GripperCommand 型に替えて
     msgbuild を無くす (判定ロジックは Isaac に残せる。懸念: cancel/feedback の転送、旧イメージ混在、
     `tmc_wrs_gazebo_worlds` の randomizer の配り方、把持の再検証)。
  3. msgbuild を残す場合の建て方 (ビルド済みを commit するか、Isaac の python で建てるか)。
  現状は msgbuild (`control_msgs` 5.10.0 + `tmc_control_msgs`) のままで動作確認済み。
- 検証済み (2026-10-12、`make up localhost TASK=gpsr`): ros2 コンテナ 127 パッケージのビルド
  (`ros2_laser_scan_matcher` はヘッダ名を `.hpp` に置換して通過)、move_group 起動 ("You can start planning now")、
  FJT 移動/cancel×2/NaN 拒否、`/gripper_controller/grasp`、台車 cmd_vel 走行 (0.35 m)、wrench 40 Hz、カメラ/compressed/
  camera_info 20 Hz、`controller_state` (feedback 5 関節)、`hsrb_interface` の import。
- Humble 分岐削除後 (2026-10-12) の `make build` は未実施。Dockerfile の変更は分岐の除去だけで Jazzy 側のコマンド列は同じ。
- 未検証: MoveIt の whole_body プランニングを経由した軌道 (Humble でも未実施)、tidyup 競技モードと観戦カメラ録画の Jazzy 版、
  `examples/` の実走、Apptainer (hma2_ws) 側 Humble との DDS 相互運用 (Humble ⇄ Jazzy は RMW が同じ CycloneDDS なら
  トピック互換だが、`control_msgs` の controller_state のようにメッセージ定義が変わったものは通らない)。

