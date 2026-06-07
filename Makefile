# ============================================================================
# HSR-Omniverse Makefile
#
# Usage:
#   make {ros1|ros2} <action> [pc]
#
#   ros1, ros2    Select ROS version (default: ros2 when omitted)
#   build         Build images
#   up            Start the stack
#   down          Stop and remove containers
#   logs          Tail logs of all services
#   ps            Container status
#   ros           Open bash in ros container (ros-noetic / ros2)
#   isaacsim      Open bash in isaacsim container
#   pc            CycloneDDS cross-PC mode (ros2 only, runtime only)
#
# Examples:
#   make ros2 build
#   make ros2 up
#   make ros2 up pc                  # PC mode, ROS_DOMAIN_ID=55 default
#   ROS_DOMAIN_ID=49 make ros2 up pc # override Domain ID
#   make ros1 build
#   make ros1 up
# ============================================================================

ROS_DOMAIN_ID_PC ?= 55

# スポーンするロボット切替。`make ros2 up robot=hsrb` のように小文字 robot= でも、
# ROBOT=hsrb でも渡せる。未指定なら空 → placement.yaml の robot.model に従う。
ROBOT ?= $(robot)

# --- ROS version selection (from goals) -------------------------------------
ROS := 2
ifneq (,$(filter ros1,$(MAKECMDGOALS)))
  ROS := 1
endif
ifneq (,$(filter ros2,$(MAKECMDGOALS)))
  ROS := 2
endif

ifeq ($(ROS),1)
  COMPOSE_FILE := env_docker/docker-compose.yml
  ROS_SERVICE  := ros-noetic
else
  COMPOSE_FILE := env_docker/docker-compose-ros2.yml
  ROS_SERVICE  := ros2
endif

COMPOSE := docker compose -f $(COMPOSE_FILE)

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
  # 同じ端末で 'make ... dev run' を実行し、ログをそこに表示するため。
  UP_FLAGS := -d
else
  # 通常起動はイメージ内のフラット配置。
  LAUNCH_PY := /app/launch_isaacsim.py
  # 通常起動はフォアグラウンド (全コンテナのログを表示)。
  UP_FLAGS :=
endif

# --- pc modifier ------------------------------------------------------------
PC := $(if $(filter pc,$(MAKECMDGOALS)),1,)
ifeq ($(PC),1)
  ifneq ($(ROS),2)
    $(error 'pc' modifier requires ros2 (got ros$(ROS)). CycloneDDS PC mode is ROS2-only.)
  endif
  ENV_PC := CYCLONEDDS_URI=file:///cyclonedds.pc.xml ROS_DOMAIN_ID=$${ROS_DOMAIN_ID:-$(ROS_DOMAIN_ID_PC)}
else
  ENV_PC :=
endif

# --- Targets ----------------------------------------------------------------
.PHONY: help ros1 ros2 pc dev build up down logs ps ros isaacsim run list-anims
.DEFAULT_GOAL := help

help:
	@echo "Usage: make {ros1|ros2} <action> [pc]"
	@echo ""
	@echo "Modifiers (pick one ROS version; pc is optional, ros2 only):"
	@echo "  ros1            ROS1 Noetic (compose: env_docker/docker-compose.yml)"
	@echo "  ros2            ROS2 Humble (compose: env_docker/docker-compose-ros2.yml) [default]"
	@echo "  pc              CycloneDDS cross-PC mode, ROS_DOMAIN_ID=$(ROS_DOMAIN_ID_PC) (override via env)"
	@echo "  dev             Dev mode: isaacsim stays idle (no auto Python); run it manually"
	@echo ""
	@echo "Actions:"
	@echo "  build           Build images"
	@echo "  up              Start the stack"
	@echo "  down            Stop and remove containers"
	@echo "  logs            Tail logs"
	@echo "  ps              Container status"
	@echo "  ros             Open bash in ros container ($(ROS_SERVICE))"
	@echo "  isaacsim        Open bash in isaacsim container"
	@echo "  run             (dev) Run launch_isaacsim.py manually in isaacsim"
	@echo ""
	@echo "Examples:"
	@echo "  make ros2 build"
	@echo "  make ros2 up"
	@echo "  make ros2 up TASK=hri   # configs/tasks/hri/ の設定で起動"
	@echo "  make ros2 up robot=hsrb     # ロボットを hsrb で起動 (placement.yaml より優先)"
	@echo "  make ros2 up robot=hsrc_ex  # ロボットを hsrc_ex で起動"
	@echo "  make ros2 up pc"
	@echo "  ROS_DOMAIN_ID=49 make ros2 up pc"
	@echo "  make ros1 build"
	@echo "  make ros1 up"
	@echo ""
	@echo "  # Dev mode (開発用。リリース後は up を使う):"
	@echo "  make ros2 dev up      # start containers in background (-d), isaacsim idle, prompt returns"
	@echo "  make ros2 dev run     # same terminal: run the sim, traceback/logs shown here"
	@echo "  make ros2 dev down    # stop when finished"
	@echo ""
	@echo "Resolved: ROS=$(ROS), PC=$(if $(PC),on,off), compose=$(COMPOSE_FILE)"

# Modifier "targets" — accept as no-ops so they can sit on the command line
ros1 ros2 pc dev:
	@:

# TASK=<名前> を付けると configs/tasks/<名前>/ の world/placement/dressing を使う。
#   例: make ros2 dev run TASK=hri   (未指定なら configs 直下の既定)
run:
	$(COMPOSE) exec -e TASK=$(TASK) -e ROBOT=$(ROBOT) isaacsim /ros_entrypoint.sh /isaac-sim/python.sh $(LAUNCH_PY)

# アセットサーバにある「人のアニメ」ファイル一覧を表示する確認用 (画面なし)。
#   make ros2 dev list-anims   (先に 'make ros2 dev up' でコンテナ起動が必要)
list-anims:
	$(COMPOSE) exec isaacsim /ros_entrypoint.sh /isaac-sim/python.sh /app/scripts/list_people_anims.py

build:
	$(ENV_PC) $(COMPOSE) build

# TASK=<名前> を付けると configs/tasks/<名前>/ の設定で起動する。
#   例: make ros2 up TASK=hri   (未指定なら configs 直下の既定)
up:
	TASK=$(TASK) ROBOT=$(ROBOT) $(ENV_PC) $(COMPOSE) up $(UP_FLAGS)

down:
	$(COMPOSE) down

logs:
	$(COMPOSE) logs -f

ps:
	$(COMPOSE) ps -a

ros:
	$(COMPOSE) exec $(ROS_SERVICE) bash

isaacsim:
	$(COMPOSE) exec isaacsim bash
