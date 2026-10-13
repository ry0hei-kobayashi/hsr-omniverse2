# 1 台の PC で Isaac Sim + ros2 スタックを複数同時起動する (設計メモ、未実装)

作成: 2026-10-12。状態: **設計のみ。コードは未変更。**

決定済みの要件:
- 1 インスタンス = Isaac コンテナ + ros2 (MoveIt) コンテナ + cacheproxy の 1 組。インスタンスごとに ROS_DOMAIN_ID を分けて完全に独立させる。
- 表示は GUI ウィンドウを複数出す (headless / livestream は扱わない)。
- インスタンスは番号 `N` で指定する (`make up N=2`)。`N=1` は現状と完全互換。

## 1. 現状で衝突するもの (読み取り調査の結果)

### 必ず直す
| 項目 | 場所 | 何が起きるか |
|---|---|---|
| compose project 名が固定 `-p hsr-omniverse` | `Makefile:62` | 2 つ目の `make up` が 1 つ目のコンテナ (`hsr-omniverse-isaacsim-1` 等) を乗っ取る。`down/logs/ps/exec` も 1 つしか見えない |
| `ROS_DOMAIN_ID` 既定 26 が固定 | `Makefile:49`、`docker-compose.yml:26,87` | host network なので同じ domain の 2 スタックが 1 つの ROS グラフに合流 (`/clock` が 2 本、`isaac_sim_hsr` 等のノード名重複) |
| squid が `http_port 3128` 固定 | `Dockerfile.cacheproxy:12`、`docker-compose.yml:77` (`http_proxy=http://127.0.0.1:3128`)、`network_mode: host` | 2 つ目の squid は bind できず終了。Isaac は 1 つ目の proxy を黙って使い、1 つ目が落ちると assets が取れなくなる。`TIME=` 付き (`--abort-on-container-exit`) だと squid 終了で 2 つ目のスタックごと落ちる |
| `pc` モードが `ROS_DOMAIN_ID := $(ROS_DOMAIN_ID_PC)` で上書き | `Makefile:101` | README 記載の `ROS_DOMAIN_ID=49 make up pc` が効かず常に 55 (既存バグ)。pc インスタンスを 2 つ起動すると両方 55 |

### 直した方がよい
| 項目 | 場所 | 理由 |
|---|---|---|
| Kit の persistent data を共有 | `docker-compose.yml:225` (`cache/data` → `/root/.local/share/ov/data`) | ユーザ設定・永続設定を両インスタンスが書く |
| Kit のログ dir を共有 | `docker-compose.yml:226` (`logs` → `/root/.nvidia-omniverse/logs`) | ファイル名が秒のタイムスタンプなので同時起動で衝突し得る |
| 録画出力 | `docker-compose.yml:173` (`../recordings`)、`hsr_omniverse_sim/task/arena_cameras.py:458-460` (秒解像度フォルダ名)、`:98` (`/recordings/tune` 固定) | 同一 checkout で 2 インスタンスが競技モードだと上書き |
| DDS の `MaxAutoParticipantIndex` 120 | `assets/cyclonedds.xml`、`cyclonedds.localhost.xml:54-57` | Cyclone の unicast port = 7400 + 250·domain + 10 + 2·index。index 120 で隣の domain (d+1) の discovery port にちょうど重なる。`pc.xml` は 100 |

### 共有してよい / 変更不要
- shader / ov / pip / GLCache / ComputeCache (`docker-compose.yml:220-224`)。Isaac の通常運用でも複数プロセスが共有する。
- イメージタグ `hsr-omniverse-{ros2,isaacsim}:jazzy` (同一 checkout なら同じイメージで良い)。
- X11: `DISPLAY` を渡すだけで `/tmp/.X11-unix` の mount も無い。ウィンドウ 2 枚は問題なし。
- DDS XML はポートを固定していない (`Domain id="any"`)。domain を分ければ port 範囲が分かれる。
- ノード名の固定 (`hsr.py:89` `isaac_sim_hsr`、`rgbd_republisher.py:62`、`hsr.launch.py:172,365,377`、`teleop.launch.py:74,89`) は domain 分離で足りる。
- `container_name` はどの compose にも無い (project 名だけ変えれば良い)。

## 2. 設計

### Makefile
```make
N ?= 1                                  # インスタンス番号 (正整数)
SUFFIX := $(if $(filter 1,$(N)),,-$(N))  # N=1 は空 (現状互換)
COMPOSE := docker compose -p hsr-omniverse$(SUFFIX) -f $(COMPOSE_FILE)

# domain は「既定値を先に計算して ?=」にする。pc モードの := 上書きバグも同時に直る。
ROS_DOMAIN_ID_BASE    ?= 26
ROS_DOMAIN_ID_PC_BASE ?= 55             # 旧 ROS_DOMAIN_ID_PC
ifeq ($(PC),1)
  ROS_DOMAIN_ID ?= $(shell expr $(ROS_DOMAIN_ID_PC_BASE) + $(N) - 1)
else
  ROS_DOMAIN_ID ?= $(shell expr $(ROS_DOMAIN_ID_BASE) + $(N) - 1)
endif
export ROS_DOMAIN_ID

CACHEPROXY_PORT := $(shell expr 3128 + $(N) - 1)
OV_DATA_DIR     := $(HOME)/.hsr-omniverse/cache/data$(SUFFIX)
OV_LOGS_DIR     := $(HOME)/.hsr-omniverse/logs$(SUFFIX)
RECORDINGS_DIR  := $(CURDIR)/recordings$(SUFFIX)
export CACHEPROXY_PORT OV_DATA_DIR OV_LOGS_DIR RECORDINGS_DIR
```
- `up` の先頭で上記 3 dir とキャッシュ 5 dir を `mkdir -p` する (README の手動 mkdir を不要にする)。
- `dds-check` の表示と `help` に `N=` / `CACHEPROXY_PORT` を出す。`help` に `make up N=2` / `make down N=2` / `make exec isaacsim N=2` を追加。
- 全ターゲットが `$(COMPOSE)` 経由なので、`N=` を付けるだけで `down/logs/ps/exec/run/build` が対応するインスタンスに向く。

結果: N=1 → project `hsr-omniverse`、domain 26 (pc: 55)、proxy 3128、従来パス。N=2 → `hsr-omniverse-2`、27 (56)、3129、`cache/data-2`、`logs-2`、`recordings-2/`。

### env_docker/docker-compose.yml
- `cacheproxy`: `command: ["/usr/sbin/squid", "-N", "-a", "${CACHEPROXY_PORT:-3128}"]` (`-a` は `http_port` の上書き。`Dockerfile.cacheproxy` は触らない)。
- `isaacsim.environment`: `http_proxy=http://127.0.0.1:${CACHEPROXY_PORT:-3128}`。
- `isaacsim.volumes`: `${RECORDINGS_DIR:-../recordings}:/recordings:rw`、`${OV_DATA_DIR:-${HOME}/.hsr-omniverse/cache/data}:/root/.local/share/ov/data:rw`、`${OV_LOGS_DIR:-${HOME}/.hsr-omniverse/logs}:/root/.nvidia-omniverse/logs:rw`。
- `docker-compose.dev.yml` は変更不要。

### assets/cyclonedds.xml, cyclonedds.localhost.xml
- `MaxAutoParticipantIndex` を 120 → 100 (pc.xml と揃える)。

### .gitignore
- `recordings-*/` を追加。

### docs
- README に「複数インスタンス」小節 (上記の N=2 の例、Apptainer 側は `source assets/isaac_localhost_mode.sh 27`)。
- `docs/dds-localhost.md` の複数 PC 節に「同一 PC 複数は N= で domain を分ける」を追記。

### 触らないもの
- ノード名、`arena_cameras.py` のフォルダ命名 (`recordings-N/` で回避)、イメージタグの分離、`dev` モードの `.env` 問題、`Dockerfile.ycb-grasp`。

## 3. 検証手順 (実装時)
1. `make -n up N=2 localhost` → project `hsr-omniverse-2`、`ROS_DOMAIN_ID=27`、`CACHEPROXY_PORT=3129`。`make -n up` は従来値。
2. `docker compose -p hsr-omniverse-2 -f env_docker/docker-compose.yml config` で mount 先・`http_proxy`・squid `command`。
3. 別端末で `make up localhost TASK=gpsr` と `make up localhost TASK=gpsr N=2`。
   `docker ps` に 6 コンテナ、`ss -ltnp | grep -E '3128|3129'` で squid 2 本、各 ros2 コンテナで rclpy スクリプトにより `/clock` publisher が 1 本・topic 数が単独時と同じ、N=2 側で FJT 1 本 SUCCEEDED、`make down N=2` で 1 つ目が残る。
4. `ROS_DOMAIN_ID=49 make up pc` の dry-run で 49 が通る。

## 4. 懸念 (未検証)
- **VRAM**: README の目安は 1 インスタンス 12GB。RTX 4070 (12GB) では 2 つ目が起動しないか、起動しても描画が極端に遅くなる可能性が高い。A6000 (48GB) なら 2〜3 は現実的だが未計測。`RENDER_EVERY_N_STEPS` を上げても VRAM は減らない。
- **RTF**: 単独で約 0.7 (known-issues §6)。2 つで GPU を取り合うのでさらに落ちる。競技モード (`TIME=`) はシミュ内時間で計るので時間制限自体はずれないが、実時間は伸びる。
- **Kit キャッシュの同時書き込み**: shader cache (`kit/cache`) と `ov` cache を 2 プロセスが同時に書く。Isaac 単体の通常運用では問題にならないが、初回 (コンパイル中) の同時起動は未検証。初回は 1 つずつ起動するのが安全。
- **`ipc: host`**: 両スタックがホストの `/dev/shm` を共有する。Cyclone の shared memory (iceoryx) は使っていないので DDS は影響なし。Kit / Carb が固定名の SHM を作るかどうかはリポジトリからは判断できない。
- **同一 checkout での `pc` モード複数起動**: `assets/gen_cyclonedds_pc.sh` が `assets/cyclonedds.pc.xml` を上書きするので、PEER が異なる pc インスタンスは同一 checkout では共存できない (範囲外として残す)。
- **別 checkout 同士**: イメージタグ `:jazzy` を共有するので、片方の `make build` がもう片方の実行イメージを置き換える (bind mount されないコード、`Dockerfile.isaacsim:105-136` の ADD 分、が影響)。旧 `~/hsr-omniverse` (carrobo、`carrobo-isaac`) も squid 3128 を使うので、そちらと同時起動する場合も衝突する。
- **ネットワーク負荷**: 既定の `cyclonedds.xml` (マルチキャスト) だと 1 スタック約 99 Mbps が物理 NIC に出る (dds-localhost.md)。複数起動は `make up localhost` 前提にすべき。この機体では ufw の関係で元々 localhost 必須。
- **ROS_DOMAIN_ID の範囲**: 26 + N − 1、pc は 55 + N − 1。Cyclone の既定では domain は 0〜232 まで使えるので N が数十でも問題ないが、学生環境 (hma_env2) の既定 26 と衝突しないよう N=1 以外は Apptainer 側も同じ値を source する必要がある。
- **`make tune`**: `/recordings/tune` 固定だが `recordings-N/` に分かれるので衝突しない。ただし `tune-apply` は `configs/tasks/<TASK>/placement.yaml` を書くので、同じ TASK を 2 インスタンスで tune すると後勝ち。
