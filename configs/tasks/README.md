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
| `restaurant/` | Restaurant Challenge |
