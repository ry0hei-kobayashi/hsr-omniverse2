# ============================================================================
# HSR-Omniverse Makefile
#
# Usage:
#   make <action> [dev] [pc|localhost] [nocache] [TASK=<name>] [TIME=<秒>] [SEED=<n>]
#
#   build         Build images            (make build nocache でキャッシュ無し)
#   up            Start the stack
#   down          Stop and remove containers
#   logs          Tail logs of all services
#   ps            Container status
#   ros           Open bash in ros2 container
#   isaacsim      Open bash in isaacsim container
#   exec          Open bash: make exec ros2 / make exec isaacsim
#   run           (dev) Run launch_isaacsim.py manually in isaacsim
#   tune          観戦カメラ 4 台の位置・画角を GUI で調整する (録画しない)
#   tune-apply    tune の結果を configs/tasks/<TASK>/placement.yaml に反映する
#   pc            CycloneDDS cross-PC mode (ros2 only, runtime only)
#   localhost     CycloneDDS を loopback (lo) に閉じる (LAN へのマルチキャスト漏洩対策)
#
# タスク (configs/tasks/<TASK>/task.yaml):
#   make up TASK=tidyup            configs/tasks/tidyup/ の world/placement/dressing で起動
#   make up TASK=tidyup TIME=600   競技モード: 600 秒 (シミュ内時間) で自動終了。観戦カメラ
#                                       4 台で録画して recordings/ に mp4 を保存 (task.yaml の
#                                       time_limit を上書き。arena_cameras: true のタスクで有効)
#   make up TASK=tidyup SEED=1     configs/tasks/tidyup/placement.seed1.yaml を読む
#   make tune TASK=tidyup          観戦カメラ調整。GUI で動かすと recordings/tune/ に yaml と
#                                       プレビュー画像が出る → make tune-apply TASK=tidyup
#   どのタスクでも同じキー (time_limit / arena_cameras / placement_seeds) が使える。
#   書き方は configs/tasks/README.md。
#
# 把持:
#   make up GRASP=1    アタッチ把持 (物体が指に吸い付く) を有効にする。既定は無効で、
#                           指と物体の物理接触 (摩擦) だけで掴む。
#
# Examples:
#   make build
#   make up
#   make up localhost              # DDS を lo に閉じる (docs/dds-localhost.md)
#   make up pc                     # PC mode, ROS_DOMAIN_ID=55 default
#   ROS_DOMAIN_ID=49 make up pc    # override Domain ID
#   make up RVIZ=1                 # MoveIt RViz2 も起動する (既定オフ)
# ============================================================================

ROS_DOMAIN_ID_PC ?= 55

# Isaac 側と Apptainer (hma2_ws) 側で必ず一致させる値。make up ROS_DOMAIN_ID=31 で変えられる。
# 学生環境 (hma_env2) の既定が 26 なので合わせている。
ROS_DOMAIN_ID ?= 26
export ROS_DOMAIN_ID

# スポーンするロボット切替。`make up robot=hsrb` のように小文字 robot= でも、
# ROBOT=hsrb でも渡せる。未指定なら空 → placement.yaml の robot.model に従う。
ROBOT ?= $(robot)

# --- compose file -------------------------------------------------------------
COMPOSE_FILE := env_docker/docker-compose.yml
ROS_SERVICE  := ros2

# -p hsr-omniverse: compose のプロジェクト名。付けないとフォルダ名 (env_docker) が
# プロジェクト名になり、他リポジトリのイメージ (env_docker-isaacsim 等) と衝突・上書きする。
COMPOSE := docker compose -p hsr-omniverse -f $(COMPOSE_FILE)

# --- dev modifier -----------------------------------------------------------
# 'dev' を付けると docker-compose.dev.yml を重ねて適用する。
# isaacsim は 'sleep infinity' で待機起動し、Python は自動実行されない。
# その後 'make ... dev run' で手動実行する (ログがその端末だけに出る)。
DEV := $(if $(filter dev,$(MAKECMDGOALS)),1,)
ifeq ($(DEV),1)
  # --env-file を明示しないと .env を env_docker/ 配下で探してしまい、
  # リポジトリ直下の .env (INTERIOR_AGENT_PATH 等) が読まれない。
  COMPOSE := $(COMPOSE) -f docker-compose.dev.yml --env-file .env
  # dev では scripts/ をディレクトリごと live マウントするので、そちらを実行する
  # (単一ファイル mount の inode 固定問題を回避し、編集が即反映される)。
  LAUNCH_PY := /app/scripts/launch_isaacsim.py
  # dev では up をバックグラウンド(-d)にし、端末をすぐ返す。
  UP_FLAGS := -d
else
  # 通常起動はイメージ内のフラット配置。
  LAUNCH_PY := /app/launch_isaacsim.py
  # 通常起動はフォアグラウンド (全コンテナのログを表示)。
  UP_FLAGS :=
  # 競技モード (TIME=秒) のときは、シミュレータが自動終了したら
  # 他のコンテナ (ros2 等) も一緒に止めて make up 自体を終わらせる。
  ifneq ($(TIME),)
    UP_FLAGS += --abort-on-container-exit
  endif
endif

# --- pc / localhost modifier (DDS プロファイル選択) ---------------------------
# 既定: assets/cyclonedds.xml (autodetermine NIC + マルチキャスト)
# pc:        assets/cyclonedds.pc.xml (別 PC と unicast peer。scripts/gen_cyclonedds_pc.sh で生成)
# localhost: assets/cyclonedds.localhost.xml (lo 固定・マルチキャスト無効。docs/dds-localhost.md)
PC := $(if $(filter pc,$(MAKECMDGOALS)),1,)
LOCALHOST := $(if $(filter localhost,$(MAKECMDGOALS)),1,)
ifeq ($(PC)$(LOCALHOST),11)
  $(error 'pc' と 'localhost' は同時に指定できません)
endif
ifeq ($(PC),1)
  CYCLONEDDS_URI := file:///cyclonedds.pc.xml
  ROS_DOMAIN_ID := $(ROS_DOMAIN_ID_PC)
else ifeq ($(LOCALHOST),1)
  # 'localhost' と明示された以上、環境変数より優先させる (:= で上書き)。
  CYCLONEDDS_URI := file:///cyclonedds.localhost.xml
else
  CYCLONEDDS_URI ?= file:///cyclonedds.xml
endif
# Makefile 内で定義した変数は自動では export されず、compose の
# ${CYCLONEDDS_URI} 展開に届かないので明示的に export する。
export CYCLONEDDS_URI
# 検証用にコンテナ内パス -> ホスト上の assets/ のファイル名へ戻す
DDS_FILE := $(patsubst file:///%,%,$(CYCLONEDDS_URI))

# --- nocache modifier -------------------------------------------------------
# 'nocache' を付けるとレイヤキャッシュを一切使わずに焼き直す。
#   make build nocache
# 依存 (apt/pip/git) を最新で取り直したいとき、キャッシュが壊れている疑いがあるとき用。
# isaacsim イメージは 17GB あるので数十分〜1時間以上かかる。通常は付けないこと。
# ※ --pull は付けない。ベースが nvcr.io/nvidia/isaac-sim:5.1.0 (NGC 認証が要る) なので、
#    ログインしていない環境では pull 失敗でビルドごと落ちる。ベースまで取り直したいときだけ
#    手で `make build nocache PULL=--pull`。
PULL ?=
NO_CACHE ?= $(if $(filter nocache,$(MAKECMDGOALS)),1,0)
ifeq ($(NO_CACHE),1)
  BUILD_FLAGS := --no-cache $(PULL)
else
  BUILD_FLAGS :=
endif

# --- タスク / 競技モード -------------------------------------------------------
# TIME=<秒>: 競技時間 (環境変数 TASK_TIME で isaacsim コンテナに渡る。task.yaml の time_limit を上書き)
# SEED=<n>: configs/tasks/<TASK>/placement.seed<n>.yaml を読む (PLACEMENT_SEED)。
SEED ?= $(seed)
ifneq ($(strip $(SEED)),)
  ifeq ($(strip $(TASK)),)
    $(error SEED=<n> には TASK=<name> が必要です (configs/tasks/<TASK>/placement.seed<n>.yaml を読むため))
  endif
endif
# RViz2 は既定でオフ。見たいときだけ `make up RVIZ=1`。
RVIZ ?= 0
USE_RVIZ := $(if $(filter 1 true TRUE yes YES on ON,$(RVIZ)),true,false)
# アタッチ把持。既定 0 (物理接触のみ)。`make up GRASP=1` で有効。
GRASP ?= 0
# 1 にすると観戦カメラの調整モード (make tune で自動的に 1 になる)
CAMERA_TUNE ?=

# --- 台車 / アーム / 引き出しの物理パラメータ --------------------------------------
# 値はここが一次情報。compose の既定値と揃えてあり、`make up BASE_TRAJ_P_GAIN=1.0` で上書きできる。
BASE_DIRECT_DRIVE ?= 1
# 直接駆動の台車を「衝突形状を持たない kinematic anchor」に溶接して毎ステップ
# テレポートするか (1)、平面ジョイント上の動剛体として速度を書くか (0)。
# 0 は速度書き込みと D6 位置ドライブの二重制御になり、拘束ソルバー内で競合して
# 追従が崩れる (純回転で並進が 3 倍、0.5m 指令で 0.12m しか進まない等、実測は
# scripts/hsr.py のコメント参照)。既定は 1。
# 1 は本来「衝突フィードバックが無い」「走行中に arm_flex が振られる」という
# 欠点があったが、どちらも実装で解消した:
#   - 壁: anchor を進める前に PhysX の球スイープでクランプする
#         (BASE_KINEMATIC_COLLISION / BASE_COLLISION_*)
#   - 腕: anchor を kinematic target で動かし、PhysX 自身に速度を導出させる
#         (BASE_KINEMATIC_ANCHOR_MODE=usd)
# 切り分けたいときだけ `make up BASE_KINEMATIC_DRIVE=0` で従来経路へ戻せる。
BASE_KINEMATIC_DRIVE ?= 1
# kinematic 駆動時の壁クランプ。0 にすると台車が壁をすり抜ける。
BASE_KINEMATIC_COLLISION ?= 1
# クランプに使う台車の外周半径 [m] / スイープ球の高さ [m] / 手前に残す余裕 [m]。
BASE_COLLISION_RADIUS ?= 0.24
BASE_COLLISION_HEIGHT ?= 0.25
BASE_COLLISION_MARGIN ?= 0.01
# kinematic anchor の動かし方。usd = USD の xform を書いて PhysX に kinematic
# target として解釈させる (anchor が速度を持つので articulation が滑らかに運ばれ、
# arm_flex が叩かれない)。dc = dc.set_rigid_body_pose で瞬間移動 (従来動作)。
BASE_KINEMATIC_ANCHOR_MODE ?= usd
# arm_flex のドライブ力上限 [N*m]。台車搬送時の外乱に負けて落ちるなら上げる。
ARM_FLEX_MAX_FORCE ?= 300
# --- 引き出し (trofast) ---
# 段違い棚と trofast を直動ジョイントで繋いで「本物の引き出し」にする (既定 1)。
# 該当 prim (/wrc_stair_like_drawer) が world に無ければ何もしない。
DRAWER_JOINTS ?= 1
DRAWER_PULL_LIMIT ?= 0.30
DRAWER_DAMPING ?= 15.0
DRAWER_DAMPING_MAX_FORCE ?= 200.0
BASE_VELOCITY_WRITE ?= 1
BASE_TRAJ_P_GAIN ?= 2.0
BASE_TRAJ_D_GAIN ?= 0.5
BASE_TRAJ_I_GAIN ?= 4.0
BASE_TRAJ_I_LINEAR_LIMIT ?= 0.08
BASE_TRAJ_I_ANGULAR_LIMIT ?= 0.15
BASE_CMD_TAU ?= 0.10
BASE_WHEEL_ACCEL_LIMIT ?= 41.7
BASE_STEER_ACCEL_LIMIT ?= 5.0
# 台車の並進/回転の加速度上限 [m/s^2] / [rad/s^2]。実測ではここが応答を支配する。
BASE_LINEAR_ACCEL_LIMIT ?= 1.0
BASE_ANGULAR_ACCEL_LIMIT ?= 2.0
BASE_WHEEL_DRIVE_DAMPING ?= 10.0
# キャスターの転がり軸の粘性摩擦 (0 だと静止後も回り続ける)
BASE_PASSIVE_WHEEL_DAMPING ?= 0.001
BASE_WHEEL_DRIVE_MAX_FORCE ?= 10.0
BASE_STEER_DRIVE_DAMPING ?= 5.0
BASE_STEER_DRIVE_MAX_FORCE ?= 5.0
BASE_DIRECT_JOINT_DAMPING ?= 1.0
BASE_DIRECT_JOINT_MAX_FORCE ?= 1.0
BASE_DIRECT_ROOT_DAMPING ?= 2.0
BASE_BRAKE ?= 1
BASE_BRAKE_ENGAGE_LINEAR ?= 0.05
BASE_BRAKE_ENGAGE_ANGULAR ?= 0.10
BASE_BRAKE_CREEP_SPEED ?= 0.05
BASE_BRAKE_CREEP_ANGULAR ?= 0.15
BASE_BRAKE_K ?= 200000
BASE_BRAKE_C ?= 20000
BASE_BRAKE_MAX_FORCE ?= 20000
BASE_BRAKE_ANGULAR_K ?= 50000
BASE_BRAKE_ANGULAR_C ?= 5000
BASE_BRAKE_MAX_TORQUE ?= 5000
BASE_SETPOINT_MAX_LAG ?= 0.02
BASE_SETPOINT_MAX_LAG_ANGULAR ?= 0.05
BASE_GOAL_VELOCITY_TOLERANCE ?= 0.10
BASE_JOINT_BRAKE ?= 1
BASE_DIAG ?= 0

# カメラ publish [Hz] = 60 / RENDER_EVERY_N_STEPS / (CAMERA_FRAME_SKIP + 1) * RTF
# 既定 60/2/1 = 30 Hz (実機の HSR と同等)。物理と whole-body 制御は常に 60 Hz。
# RTF が 1.0 を保てないときは CAMERA_FRAME_SKIP=1 (15 Hz) に落とす。
CAMERA_FRAME_SKIP ?= 0
RENDER_EVERY_N_STEPS ?= 2
# head_l/head_r ステレオ (1280x960 x2) は描画予算の 73% を食うのに購読者が居ないので既定 off。
ENABLE_STEREO_CAMERAS ?= 0
# reset_world の姿勢キャプチャをスポーン root の「サブツリー全体」に広げる (既定 1)。
RESET_DEEP_CAPTURE ?= 1

# isaacsim コンテナへ渡す環境変数の一覧。up (compose 用の変数代入) と
# run (docker compose exec の -e) で同じ一覧から生成し、片方だけ追加し忘れる
# のと、`-e -e` のような取りこぼしが起きないようにする。
SIM_ENV_NAMES = \
  TASK ROBOT \
  PLACEMENT_SEED CAMERA_TUNE GRASP_ATTACH RESET_DEEP_CAPTURE \
  BASE_DIRECT_DRIVE BASE_KINEMATIC_DRIVE BASE_KINEMATIC_COLLISION \
  BASE_COLLISION_RADIUS BASE_COLLISION_HEIGHT BASE_COLLISION_MARGIN \
  BASE_KINEMATIC_ANCHOR_MODE ARM_FLEX_MAX_FORCE \
  DRAWER_JOINTS DRAWER_PULL_LIMIT DRAWER_DAMPING DRAWER_DAMPING_MAX_FORCE \
  BASE_VELOCITY_WRITE BASE_TRAJ_P_GAIN BASE_TRAJ_D_GAIN BASE_TRAJ_I_GAIN \
  BASE_TRAJ_I_LINEAR_LIMIT BASE_TRAJ_I_ANGULAR_LIMIT BASE_CMD_TAU \
  BASE_WHEEL_ACCEL_LIMIT BASE_STEER_ACCEL_LIMIT BASE_LINEAR_ACCEL_LIMIT BASE_ANGULAR_ACCEL_LIMIT \
  BASE_WHEEL_DRIVE_DAMPING BASE_PASSIVE_WHEEL_DAMPING BASE_WHEEL_DRIVE_MAX_FORCE \
  BASE_STEER_DRIVE_DAMPING BASE_STEER_DRIVE_MAX_FORCE \
  BASE_DIRECT_JOINT_DAMPING BASE_DIRECT_JOINT_MAX_FORCE BASE_DIRECT_ROOT_DAMPING \
  BASE_BRAKE BASE_BRAKE_ENGAGE_LINEAR BASE_BRAKE_ENGAGE_ANGULAR \
  BASE_BRAKE_CREEP_SPEED BASE_BRAKE_CREEP_ANGULAR \
  BASE_BRAKE_K BASE_BRAKE_C BASE_BRAKE_MAX_FORCE \
  BASE_BRAKE_ANGULAR_K BASE_BRAKE_ANGULAR_C BASE_BRAKE_MAX_TORQUE \
  BASE_SETPOINT_MAX_LAG BASE_SETPOINT_MAX_LAG_ANGULAR BASE_GOAL_VELOCITY_TOLERANCE \
  BASE_JOINT_BRAKE BASE_DIAG \
  CAMERA_FRAME_SKIP RENDER_EVERY_N_STEPS ENABLE_STEREO_CAMERAS
PLACEMENT_SEED := $(SEED)
GRASP_ATTACH := $(GRASP)
TASK_TIME := $(TIME)
SIM_ENV_ASSIGNMENTS = TASK_TIME=$(TASK_TIME) USE_RVIZ=$(USE_RVIZ) $(foreach v,$(SIM_ENV_NAMES),$(v)=$($(v)))
SIM_ENV_EXEC_FLAGS = -e TASK_TIME=$(TASK_TIME) $(foreach v,$(SIM_ENV_NAMES),-e $(v)=$($(v)))

# --- Targets ----------------------------------------------------------------
.PHONY: help pc localhost dev nocache dds-check build up down logs ps ros isaacsim exec run tune tune-apply list-anims offline-assets
.DEFAULT_GOAL := help

help:
	@echo "Usage: make <action> [dev] [pc|localhost] [nocache] [TASK=<name>] [TIME=<秒>] [SEED=<n>]"
	@echo ""
	@echo "Modifiers:"
	@echo "  pc              CycloneDDS cross-PC mode, ROS_DOMAIN_ID=$(ROS_DOMAIN_ID_PC) (override via env)"
	@echo "  localhost       CycloneDDS を loopback に閉じる (LAN へのマルチキャスト漏洩対策)"
	@echo "  dev             Dev mode: isaacsim stays idle (no auto Python); run it manually"
	@echo "  nocache         build でレイヤキャッシュを使わない"
	@echo ""
	@echo "Actions:"
	@echo "  build           Build images"
	@echo "  up              Start the stack"
	@echo "  down            Stop and remove containers"
	@echo "  logs            Tail logs"
	@echo "  ps              Container status"
	@echo "  ros             Open bash in ros2 container"
	@echo "  isaacsim        Open bash in isaacsim container"
	@echo "  exec            Open bash: make exec ros2 / make exec isaacsim"
	@echo "  run             (dev) Run launch_isaacsim.py manually in isaacsim"
	@echo "  tune            観戦カメラの位置・画角を GUI で調整する (TASK= 必須、録画しない)"
	@echo "  tune-apply      tune の結果を configs/tasks/<TASK>/placement.yaml に反映する"
	@echo "  offline-assets  人など『アセットサーバ依存』USDをusd/isaac_offline/へ取得(要ネット・会場前に一度だけ)"
	@echo ""
	@echo "Examples:"
	@echo "  make build"
	@echo "  make up"
	@echo "  make up TASK=gpsr                 # configs/tasks/gpsr/ の設定で起動"
	@echo "  make up TASK=tidyup TIME=600      # 競技モード (600 秒で自動終了・録画)"
	@echo "  make up TASK=tidyup SEED=1        # 固定 Seed 1 の配置で起動"
	@echo "  make tune TASK=tidyup             # 観戦カメラ調整 → make tune-apply TASK=tidyup"
	@echo "  make up robot=hsrb                # ロボットを hsrb で起動 (placement.yaml より優先)"
	@echo "  make up robot=hsrc_ex             # ロボットを hsrc_ex で起動"
	@echo "  make up localhost                 # DDS を lo に閉じる"
	@echo "  make up localhost ROS_DOMAIN_ID=31  # Apptainer 側と同じ値を指定する"
	@echo "  make up pc / ROS_DOMAIN_ID=49 make up pc"
	@echo "  make up RVIZ=1                    # MoveIt RViz2 も起動する (既定はオフ)"
	@echo "  make up GRASP=1                   # アタッチ把持を有効にする"
	@echo "  make up BASE_TRAJ_P_GAIN=1.0      # whole_body 台車 FB を弱める"
	@echo "  make up DRAWER_JOINTS=0           # 引き出し化をやめ『置いてあるだけの箱』に戻す"
	@echo ""
	@echo "  # Dev mode (開発用。リリース後は up を使う):"
	@echo "  make dev up      # start containers in background (-d), isaacsim idle, prompt returns"
	@echo "  make dev run     # same terminal: run the sim, traceback/logs shown here"
	@echo "  make dev down    # stop when finished"
	@echo ""
	@echo "Resolved: DDS=$(CYCLONEDDS_URI), ROS_DOMAIN_ID=$(ROS_DOMAIN_ID), compose=$(COMPOSE_FILE)"

# Modifier "targets" — accept as no-ops so they can sit on the command line.
# ('ros2' は `make exec ros2` の引数として、また旧 `make ros2 up` 互換のために残す)
ros2 pc localhost dev nocache:
	@:

# 起動前に DDS プロファイルの指定と Seed ファイルを検証する。
# CYCLONEDDS_URI を手打ちして打ち間違えると、Cyclone は
# "can't open configuration file ..." を1行出すだけで先へ進み、その後
# rmw_create_node が失敗 -> Isaac 側は rclpy.node.Node() で例外 -> segfault
# という極めて分かりにくい落ち方をする。ここでコンテナを起動する前に弾く。
dds-check:
	@case '$(CYCLONEDDS_URI)' in \
	  file:///cyclonedds.xml|file:///cyclonedds.localhost.xml|file:///cyclonedds.pc.xml) ;; \
	  *) echo '❌ CYCLONEDDS_URI が不正です: $(CYCLONEDDS_URI)'; \
	     echo '   compose が bind mount しているのは次の3つだけです:'; \
	     echo '     file:///cyclonedds.xml            (既定/マルチキャスト版)'; \
	     echo '     file:///cyclonedds.localhost.xml  (make up localhost)'; \
	     echo '     file:///cyclonedds.pc.xml         (make up pc)'; \
	     echo '   手打ちせず make の modifier を使ってください'; \
	     exit 1;; \
	esac
	@test -f 'assets/$(DDS_FILE)' || { \
	  echo '❌ assets/$(DDS_FILE) が見つかりません'; exit 1; }
	@echo '🔧 DDS: $(CYCLONEDDS_URI)   ROS_DOMAIN_ID=$(ROS_DOMAIN_ID)   TASK=$(TASK)'
	@if [ -n '$(SEED)' ]; then \
	  test -f 'configs/tasks/$(TASK)/placement.seed$(SEED).yaml' || { \
	    echo '❌ Seed ファイルが見つかりません: configs/tasks/$(TASK)/placement.seed$(SEED).yaml'; \
	    exit 1; \
	  }; \
	  echo '🎲 Placement seed: $(SEED)'; \
	fi

# TASK=<名前> を付けると configs/tasks/<名前>/ の world/placement/dressing を使う。
#   例: make dev run TASK=hri   (未指定なら configs 直下の既定)
run:
	$(COMPOSE) exec $(SIM_ENV_EXEC_FLAGS) isaacsim /ros_entrypoint.sh /isaac-sim/python.sh $(LAUNCH_PY)

# アセットサーバにある「人のアニメ」ファイル一覧を表示する確認用 (画面なし)。
#   make dev list-anims   (先に 'make dev up' でコンテナ起動が必要)
list-anims:
	$(COMPOSE) exec isaacsim /ros_entrypoint.sh /isaac-sim/python.sh /app/scripts/list_people_anims.py

# 会場(WiFiなし)対策: 人(キャラ+アニメ)など「アセットサーバ依存」の USD を依存ごと
# usd/isaac_offline/ にミラー(コピー)してくる。★ネットに繋がる環境で会場前に一度だけ実行。
# 取得後は usd/isaac_offline/ をコミットすれば、以後 people は完全オフラインで動く。
# 追加で取りたいサーバパスがあれば FETCH_ARGS で渡せる:
#   make offline-assets FETCH_ARGS='--path /NVIDIA/Assets/ArchVis/.../Table.usd'
offline-assets:
	$(COMPOSE) run --rm --no-deps \
	  -e http_proxy= -e https_proxy= \
	  -v $(CURDIR)/usd:/out_usd:rw \
	  -v $(CURDIR)/scripts/fetch_isaac_offline_assets.py:/app/fetch_isaac_offline_assets.py:ro \
	  --entrypoint bash isaacsim -lc 'L=$$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1); \
	    PYTHONPATH=$$L LD_LIBRARY_PATH=$$L/bin \
	    /isaac-sim/kit/python/bin/python3 /app/fetch_isaac_offline_assets.py \
	      --out /out_usd/isaac_offline $(FETCH_ARGS)'

build:
	$(COMPOSE) build $(BUILD_FLAGS)

# TASK=<名前> を付けると configs/tasks/<名前>/ の設定で起動する。
#   例: make up TASK=hri   (未指定なら configs 直下の既定)
up: dds-check
	@if [ "$(PC)" = "1" ] && [ -n "$(PEER)" ]; then \
	  echo "[pc] cyclonedds.pc.xml を生成 (PEER=$(PEER))"; \
	  ./scripts/gen_cyclonedds_pc.sh $(PEER); \
	elif [ "$(PC)" = "1" ]; then \
	  echo "[pc] PEER 未指定 → 既存の assets/cyclonedds.pc.xml を使用 (自動生成するなら: make up pc PEER=192.168.0.10)"; \
	fi
	$(SIM_ENV_ASSIGNMENTS) $(COMPOSE) up $(UP_FLAGS)

down:
	$(COMPOSE) down

logs:
	$(COMPOSE) logs -f

ps:
	$(COMPOSE) ps -a

ros:
	$(COMPOSE) exec $(ROS_SERVICE) /ros_entrypoint.sh bash

isaacsim:
	$(COMPOSE) exec isaacsim bash

# --- exec: コンテナに入る ----------------------------------------------------
#   make exec ros2       ROS 2 コンテナ (/ros_entrypoint.sh 経由で ROS 環境 source 済み)
#   make exec isaacsim   Isaac Sim コンテナ
# ('ros2' は ROS 版選択の modifier と同じ単語なので、exec と並べたときだけ入り先として解釈する)
EXEC_SERVICE := $(if $(filter isaacsim,$(MAKECMDGOALS)),isaacsim,$(if $(filter exec,$(MAKECMDGOALS)),$(ROS_SERVICE),))
exec:
ifeq ($(EXEC_SERVICE),isaacsim)
	$(COMPOSE) exec isaacsim bash
else ifneq ($(EXEC_SERVICE),)
	$(COMPOSE) exec $(EXEC_SERVICE) /ros_entrypoint.sh bash
else
	@echo "Usage: make exec ros2 | make exec isaacsim"; exit 1
endif

# ============================================================
# カメラ調整モード (どのタスクでも使える。TASK= 必須)
# ============================================================
# 観戦カメラ 4 台だけ作って起動する (録画も自動終了もしない)。
# Isaac Sim の Viewport 左上のカメラ選択で Camera_north などを選ぶと、その
# カメラの映像がそのまま見える。動かすたびに次の 2 つが書き出される:
#   recordings/tune/arena_cameras.yaml  <- configs/tasks/<TASK>/placement.yaml に貼る設定
#   recordings/tune/preview.png         <- 実際に録画される 2x2 の絵
# (dev モードで使いたいときは `make dev up CAMERA_TUNE=1 TASK=...` + `make dev run TASK=...`)
# 子 make には MAKECMDGOALS の modifier が伝わらないので、選ばれたプロファイルを
# コマンドライン変数として明示的に引き継ぐ。
tune:
	@test -n '$(TASK)' || { echo '❌ make tune TASK=<name> のように TASK を指定してください'; exit 1; }
	@$(MAKE) up CAMERA_TUNE=1 TASK=$(TASK) CYCLONEDDS_URI=$(CYCLONEDDS_URI) ROS_DOMAIN_ID=$(ROS_DOMAIN_ID)

# tune の調整結果 (recordings/tune/arena_cameras.yaml) を
# configs/tasks/<TASK>/placement.yaml に反映する。make tune だけでは反映されない
# (コンテナから configs/ は読み取り専用のため)。
tune-apply:
	@test -n '$(TASK)' || { echo '❌ make tune-apply TASK=<name> のように TASK を指定してください'; exit 1; }
	@TASK=$(TASK) python3 scripts/apply_tune.py
