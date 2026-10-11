# HSR-Omniverse

NVIDIA Omniverse / Isaac Sim 上で HSR を扱うためのリポジトリ。

> **Note:** **ROS 2 Humble + Isaac Sim 5.1** (CUDA 12.8 世代のドライバ向け)。ROS1 (noetic) 構成は削除済み。

Isaac Sim 5.1 は Ubuntu 24.04 / Python 3.11 ベースで、ROS 2 Humble の rclpy を**同梱**している。
このリポジトリは apt で ROS を入れず、同梱の rclpy と、同梱に無いメッセージ
(`control_msgs` / `tmc_control_msgs` / `gazebo_msgs` / `tf_transformations`) を Python 3.11 向けに
建てたもの (`env_docker/Dockerfile.isaacsim` の msgbuild ステージ) を使う。
MoveIt など ROS 側のスタックは従来どおり 22.04 + Humble の `ros2` コンテナで動き、ホストネットワーク上の DDS で繋がる。

---

## クイックスタート (ROS 2 Humble + Isaac Sim 5.1)

### 動作確認済み環境

| 項目                     | バージョン                  |
| ------------------------ | --------------------------- |
| OS (ホスト)              | Ubuntu 22.04 / 24.04        |
| GPU                      | NVIDIA RTX A6000 (VRAM 48GB) / RTX 4070 (12GB) |
| NVIDIA Driver            | 580.x (CUDA 12.8 以降が必要)  |
| Docker                   | 29.x                        |
| NVIDIA Container Toolkit | latest                      |
| Isaac Sim (コンテナ内)   | 5.1.0 (Ubuntu 24.04 / Python 3.11) |
| ROS 2 (コンテナ内)       | Humble (Isaac 側は同梱 rclpy、ros2 側は 22.04 + apt) |

推奨スペック: RTX 30 系以降, VRAM 12GB 以上, RAM 32GB 以上, 空きディスク 100GB 以上。

### 1. Docker と NVIDIA Container Toolkit のインストール

```bash
# Docker
sudo apt update
sudo apt install -y docker.io docker-compose-v2

# ユーザーを docker グループに追加
sudo gpasswd -a $USER docker
# (一度ログアウト・ログインする。または現在のシェルだけなら `newgrp docker`)

# NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# コンテナから GPU が見えるか確認
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
```

### 2. リポジトリのクローン

```bash
git clone --recursive https://github.com/ry0hei-kobayashi/hsr-omniverse2.git
cd hsr-omniverse2

# --recursive を忘れた場合は:
git submodule update --init --recursive
```

サブモジュールは `usd/hsrb` (HSR の USD)、`usd/wrs_models` (YCB 物体・家具)、
`3rdparty/semu.robotics.ros2_bridge` (ROS 2 アクションサーバー。Toni-SM 氏の MIT ライセンス拡張を
Isaac 5.1 向けに移植した fork、ブランチ `isaac5.1-humble`)。どれが無くても起動しない。

### 3. キャッシュディレクトリ作成と X11 許可

```bash
# キャッシュディレクトリ(docker-compose でマウントされる)
mkdir -p ~/.hsr-omniverse/cache/{kit,ov,pip,glcache,computecache,data}
mkdir -p ~/.hsr-omniverse/logs

xhost local:
```

### 4. ビルド・起動

操作は `Makefile` 経由で統一しています。コマンドは `make [ros2] <action> [dev] [pc|localhost] [nocache] [TASK=...]` のかたちで組み合わせます (`ros2` は省略可。ROS1 構成を削除したので単語だけ互換のために受け付けます)。

```bash
# ROS2 Humble (デフォルト)
make build               # 全イメージをビルド (初回 30 分〜1 時間。isaacsim は 17GB)
make build nocache       # レイヤキャッシュ無しで焼き直す
make up                  # 起動 (ROS_DOMAIN_ID=26、DDS はマルチキャスト版 assets/cyclonedds.xml)
make up localhost        # DDS を loopback に閉じる (LAN へのマルチキャスト漏洩対策。docs/dds-localhost.md)
make up pc               # 別 PC と通信する PC モード (CycloneDDS unicast + ROS_DOMAIN_ID=55)
ROS_DOMAIN_ID=49 make up pc  # Domain ID を上書き
ROS_DOMAIN_ID=26 make up pc PEER=192.168.0.10  # 相手IP指定で pc.xml 自動生成 (下記参照)
make up TASK=gpsr        # configs/tasks/gpsr/ の world/配置/見た目で起動 (後述)
make up TASK=tidyup TIME=600 SEED=1  # 競技モード (600 秒で自動終了・観戦カメラ録画・固定 Seed 1)
make tune TASK=tidyup    # 観戦カメラの位置調整 → make tune-apply TASK=tidyup で反映
make up robot=hsrc_ex    # スポーンするロボットを切り替え (hsrb/hsrc_ex、後述)
make up RVIZ=1           # MoveIt の RViz2 も起動 (既定オフ)
make up GRASP=1          # アタッチ把持 (指に吸い付く) を有効化。既定は物理接触のみ
make down                # 停止
make ps                  # コンテナ状態
make exec ros2           # ros2 コンテナで bash (ROS 環境 source 済み)
make exec isaacsim       # isaacsim コンテナで bash
make logs                # ログを tail

# 単語の順序は問いません: make pc up ros2 でも同じ

make                          # 引数なし → help (現在の解決状態も表示)
```

裏側で参照する compose ファイル:
- `ros2` → `env_docker/docker-compose.yml`

Isaac Sim の初回起動は 10〜20 分かかります(シェーダーコンパイル、USD 読み込み)。2 回目以降はマウントしたキャッシュが効くので速くなります。

### 5. ROS 2 トピックの確認

別ターミナルから:

```bash
make exec ros2
# コンテナ内で (ROS 環境は /ros_entrypoint.sh が source 済み):
ros2 topic list
```

> **ROS_DOMAIN_ID** は既定 26 (学生環境 hma_env2 に合わせている)。別 PC や Apptainer 側と通信するときは
> 両方で必ず揃えること。ずれると一切つながらない。`make up ROS_DOMAIN_ID=31` で変更できる。

#### トラブルシューティング: トピックが `/parameter_events` と `/rosout` しか出ない

`ros2 topic list` の結果が `/parameter_events` と `/rosout` の 2 つだけになることがあります。多くの場合トピックが流れていないのではなく、**`ros2` の探索デーモン (今あるトピック/ノードを覚えておく裏方プロセス) が古い「空」の状態をキャッシュしている**ためです。Isaac Sim はロードに時間がかかる (初回はシェーダーコンパイルで 10〜20 分) ので、Isaac Sim がトピックを出し始める前にデーモンが起動すると「何も無い」と覚えたまま更新されません。

対処は 1 行。デーモンを止めれば次のコマンドで自動的に作り直され、最新の状態を取得します:

```bash
ros2 daemon stop
ros2 topic list   # 再取得
```

正しく流れていれば `/joint_states` (約 30Hz)・`/scan`・`/head_rgbd_sensor/...` (カメラ)・`/tf` や、ノード `/isaac_sim_hsr` などが見えます。確認用:

```bash
ros2 topic hz /joint_states     # 流量 (Hz) を見る
ros2 topic echo /scan --once    # 中身を 1 件だけ見る
```

> **切り分けのヒント:** isaacsim コンテナ自身と ros2 コンテナの両方から `ros2 topic list` を比べると、「コンテナ間通信の問題」か「Isaac Sim 側がまだ出していない」かを判別できます。なお Isaac Sim が完全に起動し終えてから確認すれば、最初から正しく見えることがほとんどです。

#### トラブルシューティング: `/head_rgbd_sensor/reconsted/points` が空

トピックは `ros2 topic list` に出るのに `ros2 topic echo` しても何も来ない場合。

この点群は sim 側ではなく `hma2_ws` の `head_pcl_reconst` (`hma_pcl_reconst2`) が作っている。
このノードは `use_compressed: true` で動くため、購読先は raw の画像ではなく**圧縮版**:

- `/head_rgbd_sensor/rgb/image_rect_color/compressed`
- `/head_rgbd_sensor/depth_registered/image_rect_raw/compressedDepth`

Isaac Sim の `ROS2CameraHelper` は raw の `sensor_msgs/Image` しか出さないので、これを圧縮して
配り直す `scripts/rgbd_republisher.py` (ノード名 `rgbd_republisher`) が必要。
`hsr.launch.py` が既定で起動する。これが動いていないと depth/rgb の同期が一度も発火せず、
publisher は advertise だけされて無言になる。

```bash
make exec ros2
# コンテナ内で:
ros2 node list | grep rgbd_republisher                          # 起動しているか
ros2 topic info /head_rgbd_sensor/rgb/image_rect_color/compressed  # Publisher count が 1 か
```

`Publisher count: 0` なら republisher が落ちている。`make logs | grep -i republisher` でエラーを確認する。
意図的に止めたい場合のみ `ros2 launch /hsr.launch.py use_rgbd_republisher:=false`。

RViz で表示するときは publisher が BEST_EFFORT なので **Reliability を Best Effort** にすること。
Reliable のままだと QoS 不一致で永久に繋がらない。

#### カメラのレート (既定 30 Hz)

`/head_rgbd_sensor/*` の publish レートは 2 つの間引き設定の積で決まる。

```
カメラ publish [Hz] = 60 / RENDER_EVERY_N_STEPS / (CAMERA_FRAME_SKIP + 1) * RTF
```

| 変数 | 既定 | 意味 |
|---|---|---|
| `RENDER_EVERY_N_STEPS` | 2 | 物理 60 Hz のうち何ステップに 1 回描画するか (= 30 Hz 描画) |
| `CAMERA_FRAME_SKIP` | 0 | 描画したフレームのうち何枚に 1 枚 publish するか (0 = 毎フレーム) |
| `ENABLE_STEREO_CAMERAS` | 0 | head_l/head_r ステレオ (1280x960 x2) を作るか |

既定は `60 / 2 / 1 = 30 Hz` で実機の HSR と同等。物理と whole-body 制御と `/clock` は常に 60 Hz。
ステレオ 2 本はレンダープロダクトのピクセル予算の 73% を占めるのに購読者が居ないので既定で作らない。
ステレオ画像が要るタスクのときだけ `make up ENABLE_STEREO_CAMERAS=1`。

**RTF が 1.0 を保てないとき** (`ros2 topic hz /clock` が 60 から大きく落ちるとき) は、軽い順に
`make up CAMERA_FRAME_SKIP=1` (15 Hz)、`make up RENDER_EVERY_N_STEPS=3` (20 Hz) を試す。
いずれも環境変数なので再ビルドは不要。

#### トラブルシューティング: 点群のレートが発行元より低い

`/head_rgbd_sensor/reconsted/points` は **1 枚 9.83MB** (640x480 x point_step 32) と巨大で、
CycloneDDS の UDP 断片化の影響をまともに受ける。発行元が 30 Hz でも受信側でごっそり落ちることがある。

sim 側 (docker コンテナ) の DDS 設定は `assets/cyclonedds.xml` にあり、`FragmentSize` と
`WhcHigh`、ソケットバッファ (100MB) をこのサイズ前提で調整済み。

発行元の `head_pcl_reconst` は `hma2_ws` の Apptainer 側で動いており、DDS 設定も別ファイル
(`hma2_ws/env/rmw_profiles/5e_isaac/cyclonedds.xml`、`gen_rmw_profiles.py` の自動生成物) を読む。
そちらが Cyclone 既定 (1344B / 100kB) のままだと 1 断片の欠落でサンプルが丸ごと失われる。
直すのは **生成スクリプト側** (hma2_ws リポジトリの作業)。

#### DDS を loopback に閉じる (`make up localhost`)

既定の `assets/cyclonedds.xml` は物理 NIC にバインドし、点群・画像を LAN へマルチキャストで撒く
(実測で Wi-Fi が 500 Mbps → 50 Mbps に低下)。同一ホスト内だけで ROS 通信するなら
`make up localhost` で `assets/cyclonedds.localhost.xml` (lo 固定・マルチキャスト無効) に切り替える。
Apptainer 側は `source scripts/isaac_localhost_mode.sh [ROS_DOMAIN_ID]`。詳細は [docs/dds-localhost.md](./docs/dds-localhost.md)。

### 別 PC からアクセスする (CycloneDDS PC モード, ROS2 のみ)

別マシンの ROS 2 (例: HSR 実機 / Singularity を入れた操作用ラップトップ) と通信したい場合は
`make up pc` を使います。

**相手 IP を `PEER=` で渡すと `assets/cyclonedds.pc.xml` を自動生成します**(NIC 名は相手と同じ
サブネットから自動判定。手編集不要):

```bash
ROS_DOMAIN_ID=26 make up pc PEER=192.168.0.10        # 相手1台
ROS_DOMAIN_ID=26 make up pc PEER="192.168.0.10 192.168.0.11"  # 複数台
```

- `PEER=` を付けると起動前に `scripts/gen_cyclonedds_pc.sh` が走り、自分の NIC を自動判定して
  `assets/cyclonedds.pc.xml` を書き出します。NIC を明示したいときは `ISAAC_NIC=enp3s0` を併用。
- `PEER=` を省くと既存の `assets/cyclonedds.pc.xml` をそのまま使います。
- 手動生成だけしたいとき: `./scripts/gen_cyclonedds_pc.sh 192.168.0.10`
- 生成される `assets/cyclonedds.pc.xml` は**環境依存**です(git では変更扱いになるのでコミットしない)。

`pc` 修飾子は `CYCLONEDDS_URI=file:///cyclonedds.pc.xml` と `ROS_DOMAIN_ID` を export して compose を
起動します。`assets/cyclonedds.pc.xml` は常時 bind mount されているのでイメージのリビルドは不要です。

> **ROS_DOMAIN_ID は両 PC で必ず揃える**こと。`pc` の Makefile 既定は 55 ですが、操作側
> (Singularity の `5d_isaac_sim_mode.sh`) が 26 を使う場合は、Sim 側も `ROS_DOMAIN_ID=26` を付けて
> 揃えます。ずれると一切つながりません。

#### 相手がジョイスティック操作用ラップトップ (Singularity) の場合

役割分担は実機と同じで、Sim 側 (この ros2 コンテナ) がロボット本体の代わりにテレオペ
"ロジック本体"(`hsr.launch.py` の `use_teleop`、既定 ON) を動かし、ラップトップ側は通常の
bringup(joy_node + teleop 並べ替え)を動かして `/joy` を送ります。ラップトップ側は
`. 5d_isaac_sim_mode.sh --network <この PC の IP>` で domain/peer/NIC が自動設定されます
(NIC は同サブネットから自動判定、`ip` が無い環境でも python3 でフォールバック)。

---

## タスクを選んで起動する（タスク開発者向け）


> world・物体配置・見た目などの設定変更は`configs/` 側で行います。

初回のみ、上の「クイックスタート」の手順で `make build` までを済ませておきます。
あとは毎回これ 1 行です。

```bash
make up TASK=hri
```

`TASK=<名前>` で起動するタスクを選びます。使える名前:

| TASK の名前 | タスク |
|---|---|
| `hri` | Human Robot Interaction |
| `pick_and_place` | Pick and Place |
| `gpsr` | General Purpose Service Robot |
| `laundry` | Doing Laundry |
| `restaurant` | Restaurant |

```bash
make up                  # TASK を付けないと既定設定で起動
make up TASK=restaurant  # 別のタスクに切り替え
make down                # 終了 (コンテナ停止)
```

### ロボットを切り替える（hsrb / hsrc_ex）

スポーンするロボットは `robot=`（小文字）または `ROBOT=`（大文字）で選びます。`TASK=` と同じく `up` に付けます。

| `robot=` の値 | ロボット | 使う USD |
|---|---|---|
| `hsrb` | HSR-B（**既定**） | `usd/hsrb/` |
| `hsrc_ex` | HSR-C 拡張版 | `usd/hsrc/hsrc1s.usd`（`scripts/hsr_hsrc_ex.py` で起動） |

```bash
make up robot=hsrb              # HSR-B で起動（既定なので省略しても同じ）
make up robot=hsrc_ex           # HSR-C 拡張版で起動
make up TASK=hri robot=hsrc_ex  # TASK と併用も可
ROBOT=hsrc_ex make up           # 大文字の環境変数でも同じ
```

- 省略時は `hsrb` で起動します。
- 優先順位は `robot=` / `ROBOT=` ＞ 既定（`hsrb`）。

### タスクごとの設定を変えたいとき

各タスクの **world / 物体・ロボット・人の配置 / 見た目** は `configs/tasks/<名前>/` で定義します。
編集方法・フォルダの中身は **➡ [configs/tasks/README.md](./configs/tasks/README.md)** を参照してください。

- 設定ファイル全体の構成（共通の既定など）: [configs/README.md](./configs/README.md)

---

## オフライン対応（会場でネットが使えないとき）

RoboCup 会場は Wi-Fi が使えないため、このシミュレータは**完全オフライン**で動くようにしてあります。

### 何が問題だったか
これまで「人（手を振るお客さん等）」は、起動のたびに NVIDIA のアセットサーバ（ネット）から
人体モデル（`Biped_Setup.usd`）とアニメ（`*.skelanim.usd`）を取得していました。会場では
ネットが無いので、ここが動かなくなる恐れがありました。

### どう直したか
必要な人アセットを、**アセットサーバと同じフォルダ構成のまま** `usd/isaac_offline/` に
同梱（コピー）しました。`people_spawn.py` / `furniture_spawn.py` は

1. まず `usd/isaac_offline/` にファイルがあればそれを使う（＝ネット不要）
2. 無ければ従来どおりアセットサーバから取得（オンライン時のフォールバック）

の順で解決します。**アニメ（手を振る等）は「骨の動きの数値データ」ファイルで、一度ローカルに
読み込めば再生中はネット不要**です。だからモーション込みで完全オフラインになります。

```
usd/isaac_offline/                         ← アセットサーバのローカルミラー（同梱）
└── Isaac/People/
    ├── Characters/Biped_Setup.usd         ← 人体モデル（+ biped_demo/ のメッシュ・テクスチャ）
    └── Animations/*.skelanim.usd          ← 手を振る/立つ/座る 等のアニメ
```

### ミラーを作り直す・増やす（オンライン環境で一度だけ）
`usd/isaac_offline/` は、ネットに繋がる環境で次を一度実行すれば自動生成されます
（依存ファイルを再帰的にたどって漏れなく取得します）。

```bash
make offline-assets
# 追加でサーバ上の家具なども取りたいとき:
make offline-assets FETCH_ARGS='--path /NVIDIA/Assets/ArchVis/.../Table.usd'
```

中身は `scripts/fetch_isaac_offline_assets.py`、対象一覧は同ファイルの `DEFAULT_SEEDS`。
詳細は [`usd/isaac_offline/README.md`](./usd/isaac_offline/README.md)。

### ⚠️ 会場へ持ち込む前に（重要）
別の PC でクローンして使う場合、`usd/isaac_offline/`（約 134MB）が**その PC に存在している
必要があります**。方法は次のどちらか:

- **(A) リポジトリにコミットしておく**（推奨・確実）: `git add usd/isaac_offline/` してコミット。
  クローンするだけで会場でも人が出ます。※約 134MB 増えます。
- **(B) 各 PC で会場前に取得**: その PC で一度 `make offline-assets` を実行（要ネット）。

### 起動時の確認方法
起動ログに次が出ていれば、人はローカル（オフライン）から読めています:

```
[people] 人アセットはローカル同梱を使用します(オフラインOK・ネット不要): /app/usd/isaac_offline/...
```

代わりに `WARNING: ... オンライン取得にフォールバック` が出たら、`usd/isaac_offline/` が
その PC に無い状態です（上の (A) または (B) を実施してください）。

---

## ディレクトリ構成

```
.
├── 3rdparty/      # 外部由来モジュール。semu.robotics.ros2_bridge/ は git submodule (MIT, Toni-SM の fork: ROS2 アクションサーバー)、OgnROS1Action*.py は ROS1 用
├── assets/        # 設定/リソース (cyclonedds*.xml 3 種, ros_entrypoint.sh.*, hsr-omniverse.rviz, MoveIt 上書き用 joint_limits-smooth.yaml / moveit-no-octomap.rviz)
├── configs/       # 実行時設定 (placement.yaml, dressing.yaml, tasks/<task>/ …) ※下のリンク参照
├── docs/          # 補足ドキュメント (dds-localhost.md, tmc_wrs_gazebo_worlds-build-note.md)
├── env_docker/    # Dockerfile.* (isaacsim-ros2 は Isaac 5.1 + py3.11 メッセージビルドの 2 ステージ) と docker-compose*.yml
├── examples/      # 制御指示のサンプル (hsrb_interface / MoveIt)。ros2 コンテナに /examples で live マウント
├── launch/        # ROS 2 launch ファイル
├── recordings/    # 競技モードの録画・tune の出力先 (git 管理外)
├── scripts/       # 実行スクリプト (launch_isaacsim.py = エントリポイント, hsr.py + hsr_*.py = ロボット, task_config.py, drawers.py, arena_cameras.py 他)
├── usd/           # USD アセット (isaac_offline/ = オフライン用の人アセット同梱ミラー, unknown_objects/ 等)
└── worlds/        # .world ファイル (家具・壁の配置)
```

エントリポイント: `scripts/launch_isaacsim.py` (旧 `sample-ros.py`)。Isaac Sim から HSR を起動するメインスクリプト。

`scripts/` の構成 (ステップ4 で分割):

| ファイル | 役割 |
|---|---|
| `launch_isaacsim.py` | 起動の流れ本体 (world 読み込み・物体 spawn・リセット・ROS サービス・メインループ) |
| `task_config.py` | `configs/tasks/<TASK>/` の読み込みと環境変数 (`TASK_TIME` / `CAMERA_TUNE` / `PLACEMENT_SEED` / `RENDER_EVERY_N_STEPS`) の上書き |
| `drawers.py` | 段違い棚の trofast を PrismaticJoint で引き出し化 |
| `hsr.py` | `class hsr`: ROS ノード、アクションサーバ生成、joint_states / odom / wrench の publish、`step()` |
| `hsr_params.py` | `BASE_*` / `ARM_FLEX_MAX_FORCE` / `CAMERA_FRAME_SKIP` 等の環境変数パラメータ |
| `hsr_sensors.py` | カメラ / IMU / LiDAR の OmniGraph 構築 (`SensorsMixin`) |
| `hsr_physics.py` | 質量・慣性・ジョイント・ドライブ・摩擦の調整 (`PhysicsTuningMixin`) |
| `hsr_base_drive.py` | 台車駆動: kinematic anchor、ブレーキ、壁クランプ、`reset_to_spawn` (`BaseDriveMixin`) |
| `hsr_grasp.py` | アタッチ把持と物体姿勢レポート (`GraspMixin`) |
| `hsr_action_servers.py` | semu.robotics.ros2_bridge の HSR 向けサブクラス (腕/頭/台車/グリッパ) |
| `hsr_kinematics.py` / `hsr_math.py` | 台車の運動学・オドメトリ / クォータニオン補助 |
| `hsr_hsrc_ex.py` | hsrc_ex 用の差分 (`hsr.hsr` を継承) |

設定まわり (タスク別の world/配置/見た目): [configs/README.md](./configs/README.md) → タスク個別は [configs/tasks/README.md](./configs/tasks/README.md)。

既知の注意点・懸念点 (DDS の探索、非推奨 API、root 所有の録画、未検証項目など): [docs/known-issues.md](./docs/known-issues.md)。
