# io/configs/tasks/ — タスク別の設定

RoboCup@Home の各タスクごとに **world / placement / dressing / 競技モード** を切り替えるためのフォルダ。

## 使い方

```bash
make up TASK=hri                       # タスクの設定で起動
make up TASK=tidyup TIME=600 SEED=2    # 競技モード (600 秒で自動終了・録画) + 固定配置 Seed 2
make tune TASK=tidyup                  # 観戦カメラ 4 台の位置・画角を GUI で調整 (録画なし)
make tune-apply TASK=tidyup            # 調整結果を io/configs/tasks/tidyup/placement.yaml に反映
```

`TASK=<フォルダ名>` を付けると `io/configs/tasks/<フォルダ名>/` の設定が使われる。
`TASK` を付けないと、従来どおり `io/configs/` 直下の既定 (`placement.yaml` / `dressing.yaml`) が使われる。
`TIME=` / `SEED=` / `tune` / `tune-apply` は **どのタスクでも同じように使える**
(下の「競技モード・観戦カメラ・固定配置」)。

（開発中にログを分けて見たいときは `make dev up` → `make dev run TASK=hri` でも同じ。リリース後は上の `up` を使う。）

## フォルダの中身

| ファイル | 必須? | 内容 |
|---|---|---|
| `task.yaml` | 必須 | このタスクで使う **world のファイル名**、**dressing の選択** (preset / lighting)、**競技モード** (time_limit / arena_cameras / placement_seeds) |
| `placement.yaml` | 任意 | robot / objects / furniture / people / arena_cameras。**置けば上書き**、無ければ共通の `io/configs/placement.yaml` |
| `placement.seed<n>.yaml` | 任意 | 固定配置 (Seed)。`SEED=<n>` で `objects:` だけがこのファイルに差し替わる (名前は `task.yaml` の `placement_seeds` で決める) |

### `task.yaml` の書き方

```yaml
# このタスクで使う world (io/models/worlds/ 内のファイル名)
world: rc26_3330.world

# 見た目 (io/configs/dressing.yaml のプリセットから選ぶ)
dressing:
  preset: lab        # dressing_presets のどれか
  lighting: default  # lighting_presets のどれか

# ---- 競技モード (省略すると下の既定値 = 従来どおりの起動) ----
time_limit: 0          # 競技時間 (秒, シミュレータ内時間)。>0 で時間が来たら自動終了。
                       # 環境変数 TASK_TIME (make up TASK=<名前> TIME=600) で上書き。
arena_cameras: false   # true で観戦カメラ 4 台 (北/南/東/西) を作る。
                       # time_limit > 0 なら録画 (/recordings に mp4)、CAMERA_TUNE=1 (make tune) なら調整モード。
placement_seeds: placement.seed{n}.yaml   # 固定配置ファイル名のパターン。{n} が Seed 番号。
                                          # 環境変数 PLACEMENT_SEED=<n> (make up ... SEED=<n>) で選ぶ。
```

- `world` … `io/models/worlds/` に置いた `.world` ファイルの**名前だけ**書く (本体は io/models/worlds/ に置いたまま共有)。
- `dressing` … テクスチャ/照明の**ライブラリ自体は共通の `io/configs/dressing.yaml`**。ここでは「どれを使うか」を選ぶだけ。新しい見た目が必要なら `io/configs/dressing.yaml` にプリセットを追加してから名前で選ぶ。
- `time_limit` / `arena_cameras` / `placement_seeds` … 競技モードの設定 (下記)。**task.yaml が一次情報**で、環境変数はそれを上書きするだけ。

## 競技モード・観戦カメラ・固定配置 (全タスク共通)

| したいこと | 書く場所 / コマンド |
|---|---|
| 制限時間で自動終了 | `task.yaml` の `time_limit: 600`、または `make up TASK=<名前> TIME=600` (環境変数 `TASK_TIME`)。`TIME=0` で無効化 |
| 観戦カメラで録画 | `task.yaml` に `arena_cameras: true` + `time_limit > 0`。カメラ位置は `placement.yaml` の `arena_cameras:` (無ければ床の中心から自動配置)。動画は `io/recordings/<日付_時刻>/arena.mp4` |
| カメラの位置・画角を調整 | `make tune TASK=<名前>` (環境変数 `CAMERA_TUNE=1`)。録画も自動終了もせずカメラだけ作る。動かすと `io/recordings/tune/arena_cameras.yaml` と `preview.png` が書き出される |
| 調整結果を反映 | `make tune-apply TASK=<名前>` → `io/configs/tasks/<名前>/placement.yaml` の `arena_cameras:` ブロックだけを差し替える (他のセクション・コメントは残る) |
| 固定配置 (Seed) で起動 | `make up TASK=<名前> SEED=2` (環境変数 `PLACEMENT_SEED=2`) → `io/configs/tasks/<名前>/placement.seed2.yaml` の **`objects:` だけ**が使われる。ファイルが無ければエラーで止まる |

環境変数の優先順位 (すべて `sim/hsr_omniverse2_sim/launch_isaacsim.py` の先頭「タスク選択と競技モードの解決」で決まる):

- `TASK_TIME` が空でなければ `time_limit` を上書き (数値でなければ警告して task.yaml の値)。
- `CAMERA_TUNE=1` のときは `arena_cameras: false` でもカメラを作り、`time_limit > 0` でも録画・自動終了を止める (調整モードが勝つ)。
- `PLACEMENT_SEED=<n>` は `TASK` と一緒に指定する。`placement_seeds` の `{n}` を置換したファイルをタスクフォルダから探し、無ければエラー。
  Seed ファイルは `objects:` (と `round_table` のような `furniture:` 由来の置き台の定義) だけを持てばよく、
  `robot:` / `people:` / `furniture:` の実配置 / `arena_cameras:` は `placement.yaml` のものが使われる。
- `time_limit > 0` かつ `arena_cameras: false` のときは録画せず、時間が来たら終了するだけ。

例: `tidyup/` は `time_limit: 600`、`arena_cameras: true`、Seed 1〜4 を持つ。詳細は [`tidyup/README.md`](./tidyup/README.md)。

### placement を変えたいとき

そのタスクで物体やロボット位置・人を変えたいときは、`io/configs/placement.yaml` をこのフォルダに **`placement.yaml` としてコピー**して編集する。無ければ共通の既定が使われる。

## 用意されているタスク (RoboCup@Home 2026 の Tests)

| フォルダ | タスク |
|---|---|
| `hri/` | Human Robot Interaction Challenge |
| `pick_and_place/` | Pick and Place Challenge |
| `gpsr/` | General Purpose Service Robot Challenge |
| `laundry/` | Doing Laundry Challenge |
| `restaurant/` | Restaurant Challenge (本物の机・椅子 USD + 手を振る人。下記参照) |
| `tidyup/` | TidyUp (rc26)。4 部屋アリーナ + 引き出し + 競技モード (録画・Seed)。[`tidyup/README.md`](./tidyup/README.md) |

## 本物の机・椅子 (furniture) を置く

`restaurant/` は「レストランのデバッグ環境」として、箱ではなく**本物のメッシュの机・椅子**を
置いている。`placement.yaml` の **`furniture:`** セクションで位置を指定する。机・椅子は
`io/models/usd/restaurant/`（CC0・同梱）から読むのでネット不要。人 (`people:`) も会場（ネットなし）で
動くよう、人体モデルとアニメを `io/models/usd/isaac_offline/` に同梱済み（ローカルにあればそれを使い、
無ければアセットサーバへフォールバック）。詳細はリポジトリ直下 `README.md` の「オフライン対応」。

```yaml
furniture:
  - usd: restaurant/Whittershins/Whittershins.usd   # 丸テーブル(直径約1.5m, io/models/usd/restaurant/ 同梱)
    name: table_a
    x: 3.0
    y: 1.6
    yaw: 0        # 度
```

- `usd` … `io/models/usd/` からの相対パス (例 `restaurant/...`) はリポジトリ同梱のローカル USD、
  `/` 始まりはアセットサーバ上のパス、`omniverse://`/`http(s)://` はその URL。
- restaurant タスクは机・椅子を `io/models/usd/restaurant/` に、人を `io/models/usd/isaac_offline/` に同梱しているので
  **起動時のネット取得が不要**（完全オフライン）。詳細は [`io/models/usd/restaurant/README.md`](../../models/usd/restaurant/README.md)
  と [`io/models/usd/isaac_offline/README.md`](../../models/usd/isaac_offline/README.md)。
- 向き (Y-up→Z-up) と単位 (cm→m) と床への着地 (snap_to_floor) は **自動補正**されるので、
  基本は `x` / `y` / `yaw` を書くだけでよい。詳細は `restaurant/placement.yaml` の
  コメントと `sim/hsr_omniverse2_sim/env/furniture_spawn.py` を参照。
- `furniture:` を書いていないタスク (hri/gpsr 等) では何も起きない (no-op)。
