# configs/tasks/ — タスク別の設定

RoboCup@Home の各タスクごとに **world / placement / dressing** を切り替えるためのフォルダ。

## 使い方

```bash
make ros2 up TASK=hri
```

`TASK=<フォルダ名>` を付けると `configs/tasks/<フォルダ名>/` の設定が使われる。
`TASK` を付けないと、従来どおり `configs/` 直下の既定 (`placement.yaml` / `dressing.yaml`) が使われる。

（開発中にログを分けて見たいときは `make ros2 dev up` → `make ros2 dev run TASK=hri` でも同じ。リリース後は上の `up` を使う。）

## フォルダの中身

| ファイル | 必須? | 内容 |
|---|---|---|
| `task.yaml` | 必須 | このタスクで使う **world のファイル名** と **dressing の選択** (preset / lighting) |
| `placement.yaml` | 任意 | robot / objects / people。**置けば上書き**、無ければ共通の `configs/placement.yaml` |

### `task.yaml` の書き方

```yaml
# このタスクで使う world (worlds/ 内のファイル名)
world: rc26_3330.world

# 見た目 (configs/dressing.yaml のプリセットから選ぶ)
dressing:
  preset: lab        # dressing_presets のどれか
  lighting: default  # lighting_presets のどれか
```

- `world` … `worlds/` に置いた `.world` ファイルの**名前だけ**書く (本体は worlds/ に置いたまま共有)。
- `dressing` … テクスチャ/照明の**ライブラリ自体は共通の `configs/dressing.yaml`**。ここでは「どれを使うか」を選ぶだけ。新しい見た目が必要なら `configs/dressing.yaml` にプリセットを追加してから名前で選ぶ。

### placement を変えたいとき

そのタスクで物体やロボット位置・人を変えたいときは、`configs/placement.yaml` をこのフォルダに **`placement.yaml` としてコピー**して編集する。無ければ共通の既定が使われる。

## 用意されているタスク (RoboCup@Home 2026 の Tests)

| フォルダ | タスク |
|---|---|
| `hri/` | Human Robot Interaction Challenge |
| `pick_and_place/` | Pick and Place Challenge |
| `gpsr/` | General Purpose Service Robot Challenge |
| `laundry/` | Doing Laundry Challenge |
| `restaurant/` | Restaurant Challenge (本物の机・椅子 USD + 手を振る人。下記参照) |

## 本物の机・椅子 (furniture) を置く

`restaurant/` は「レストランのデバッグ環境」として、箱ではなく**本物のメッシュの机・椅子**を
置いている。人 (`people:`) と同じく Isaac 公式アセットサーバから USD を取得して配置する
仕組みで、`placement.yaml` の **`furniture:`** セクションで位置を指定する。

```yaml
furniture:
  - usd: restaurant/Whittershins/Whittershins.usd   # 丸テーブル(直径約1.5m, usd/restaurant/ 同梱)
    name: table_a
    x: 3.0
    y: 1.6
    yaw: 0        # 度
```

- `usd` … `usd/` からの相対パス (例 `restaurant/...`) はリポジトリ同梱のローカル USD、
  `/` 始まりはアセットサーバ上のパス、`omniverse://`/`http(s)://` はその URL。
- restaurant タスクは机・椅子を `usd/restaurant/` に同梱しているので**起動時のネット取得が不要**
  (人 `people:` は別途アセットサーバから取得)。詳細は [`usd/restaurant/README.md`](../../usd/restaurant/README.md)。
- 向き (Y-up→Z-up) と単位 (cm→m) と床への着地 (snap_to_floor) は **自動補正**されるので、
  基本は `x` / `y` / `yaw` を書くだけでよい。詳細は `restaurant/placement.yaml` の
  コメントと `scripts/furniture_spawn.py` を参照。
- `furniture:` を書いていないタスク (hri/gpsr 等) では何も起きない (no-op)。
