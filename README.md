# HSR-Omniverse

NVIDIA Omniverse / Isaac Sim 上で HSR を扱うためのリポジトリ。

> **Note:** **ROS1 noetic or ROS 2 Humble + Isaac Sim 4.5**

---

## クイックスタート (ROS 2 Humble + Isaac Sim 4.5)

### 動作確認済み環境

| 項目                     | バージョン                  |
| ------------------------ | --------------------------- |
| OS                       | Ubuntu 22.04                |
| GPU                      | NVIDIA RTX 4070 (VRAM 12GB) |
| NVIDIA Driver            | 580.142                     |
| Docker                   | 29.x                        |
| NVIDIA Container Toolkit | latest                      |
| Isaac Sim (コンテナ内)   | 4.5.0                       |
| ROS 2 (コンテナ内)       | Humble                      |

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
git clone --recursive https://github.com/ry0hei-kobayashi/hsr-omniverse.git
cd hsr-omniverse

# --recursive を忘れた場合は:
git submodule update --init --recursive
```

### 3. キャッシュディレクトリ作成と X11 許可

```bash
# キャッシュディレクトリ(docker-compose でマウントされる)
mkdir -p ~/.hsr-omniverse/cache/{kit,ov,pip,glcache,computecache,data}
mkdir -p ~/.hsr-omniverse/logs

xhost local:
```

### 4. ビルド・起動

操作は `Makefile` 経由で統一しています。コマンドは `make {ros1|ros2} <action> [pc]` のかたちで組み合わせます (デフォルト `ros2`、`pc` 修飾子は `ros2` 限定)。

```bash
# ROS2 Humble (デフォルト)
make ros2 build               # 全イメージをビルド
make ros2 up                  # standalone モード起動 (host ローカル、ROS_DOMAIN_ID=0)
make ros2 up pc               # PC モード (CycloneDDS PC + ROS_DOMAIN_ID=55)
ROS_DOMAIN_ID=49 make ros2 up pc  # Domain ID を上書き
make ros2 down                # 停止
make ros2 ps                  # コンテナ状態
make ros2 ros                 # ros2 コンテナで bash
make ros2 isaacsim            # isaacsim コンテナで bash
make ros2 logs                # ログを tail

# ROS1 Noetic
make ros1 build
make ros1 up
make ros1 ros                 # ros-noetic コンテナで bash

# 単語の順序は問いません: make pc up ros2 でも同じ
# 不正な組合せはエラー: make ros1 up pc → 'pc' requires ros2

make                          # 引数なし → help (現在の解決状態も表示)
```

裏側で参照する compose ファイル:
- `ros2` → `env_docker/docker-compose-ros2.yml`
- `ros1` → `env_docker/docker-compose.yml`

Isaac Sim の初回起動は 10〜20 分かかります(シェーダーコンパイル、USD 読み込み)。2 回目以降はマウントしたキャッシュが効くので速くなります。

### 5. ROS 2 トピックの確認

別ターミナルから:

```bash
make ros2 ros
# コンテナ内で:
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic list
```

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

### 別 PC からアクセスする (CycloneDDS PC モード, ROS2 のみ)

別マシンの ROS 2 (例: HSR 実機) と通信したい場合は `make ros2 up pc` を使います (ROS1 と組み合わせるとエラー)。事前に `assets/cyclonedds.pc.xml` を環境に合わせて編集してください:

- `<NetworkInterface name="..."/>` をホストの NIC 名に (`ip -br addr` で確認)
- `<Peer Address="192.168.11.5"/>` を相手 (ロボット/PC) の IP に

```bash
make ros2 up pc                       # ROS_DOMAIN_ID=55 (Makefile デフォルト)
ROS_DOMAIN_ID=49 make ros2 up pc      # Domain ID を上書き
```

`pc` 修飾子は `CYCLONEDDS_URI=file:///cyclonedds.pc.xml` と `ROS_DOMAIN_ID` を export して compose を起動します。`assets/cyclonedds.pc.xml` は常時 bind mount されているのでイメージのリビルドは不要です。

---

## タスクを選んで起動する（タスク開発者向け）


> world・物体配置・見た目などの設定変更は`configs/` 側で行います。

初回のみ、上の「クイックスタート」の手順で `make ros2 build` までを済ませておきます。
あとは毎回これ 1 行です。

```bash
make ros2 up TASK=hri
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
make ros2 up                  # TASK を付けないと既定設定で起動
make ros2 up TASK=restaurant  # 別のタスクに切り替え
make ros2 down                # 終了 (コンテナ停止)
```

### タスクごとの設定を変えたいとき

各タスクの **world / 物体・ロボット・人の配置 / 見た目** は `configs/tasks/<名前>/` で定義します。
編集方法・フォルダの中身は **➡ [configs/tasks/README.md](./configs/tasks/README.md)** を参照してください。

- 設定ファイル全体の構成（共通の既定など）: [configs/README.md](./configs/README.md)

---

## ディレクトリ構成

```
.
├── 3rdparty/      # 外部由来モジュール (OgnROS1Action*.py)
├── assets/        # 設定/リソース (cyclonedds.xml, cyclonedds.pc.xml, ros_entrypoint.sh.ros2, hsr-omniverse.rviz)
├── configs/       # 実行時設定 (placement.yaml, dressing.yaml, tasks/ …) ※下のリンク参照
├── env_docker/    # Dockerfile.* と docker-compose*.yml
├── launch/        # ROS1/ROS2 launch ファイル
├── scripts/       # 実行スクリプト (hsr.py, ros2_bridge.py, launch_isaacsim.py, sample-*.py 他)
├── usd/           # USD アセット
└── worlds/        # .world ファイル (家具・壁の配置)
```

エントリポイント: `scripts/launch_isaacsim.py` (旧 `sample-ros.py`)。Isaac Sim から HSR を起動するメインスクリプト。

設定まわり (タスク別の world/配置/見た目): [configs/README.md](./configs/README.md) → タスク個別は [configs/tasks/README.md](./configs/tasks/README.md)。
