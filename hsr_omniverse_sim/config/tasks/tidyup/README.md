# hsr_omniverse_sim/config/tasks/tidyup/ — TidyUp (rc26) タスク

4 部屋の TidyUp アリーナで「床や家具に散らかった物体を片付ける」競技のための設定。
競技モード (制限時間 + 観戦カメラ録画) と固定配置 (Seed) をこのフォルダだけで扱う。

```bash
make up TASK=tidyup                 # 通常起動 (task.yaml の time_limit=600 で競技モード・録画あり)
make up TASK=tidyup TIME=0          # 時間無制限・録画なしで開発する
make up TASK=tidyup SEED=2          # 競技用の固定配置 Seed 2 で起動
make up TASK=tidyup TIME=600 SEED=2 # Seed 2 + 600 秒の競技録画
make tune TASK=tidyup               # 観戦カメラ 4 台の位置・画角を GUI で調整 (録画なし)
make tune-apply TASK=tidyup         # 調整結果を hsr_omniverse_sim/config/tasks/tidyup/placement.yaml に反映
```

## フォルダの中身

| ファイル | 内容 |
|---|---|
| `task.yaml` | world = `rc26_tidyup.world`、dressing = `rc26_venue`、`time_limit: 600`、`arena_cameras: true`、`placement_seeds: placement.seed{n}.yaml` |
| `placement.yaml` | 通常配置。`robot:` / `objects:` / `furniture:` (ソファ・丸テーブル・椅子) / `people:` / `arena_cameras:` |
| `placement.seed1.yaml` 〜 `placement.seed4.yaml` | 競技用の固定配置。`SEED=<n>` で **`objects:` だけ**がこのファイルに差し替わる |
| `object_list.yaml` | 競技の Object List (4 カテゴリ + 未知物体 + 障害物) と、全 Seed 共通のスポーン位置の正本。プログラムからは読まれない (Seed を作る/確認するための資料) |

## アリーナ (hsr_omniverse_sim/data/worlds/rc26_tidyup.world)

座標系: **ロボットのスタート地点が原点 (0,0)、正面が +X**。Isaac のワールド座標と ROS の
odom / map 座標が一致するので、placement.yaml に書いた座標をそのままナビゲーションのゴールに使える。
`robot:` は (0, 0, 0) のまま動かさず、スタート位置を変えたいときは world 側を平行移動する
(ずれていると起動ログに `[hsr] WARNING: robot: が原点 (0, 0, 0) ではありません` が出る)。

- 外周 8.0 x 8.0 m (X 0.4〜8.4, Y -2.4〜5.6)、壁の高さ 0.6 m (`wall_*`)
- 田の字に 4 部屋 (各 4 x 4 m)。仕切り壁のドア開口は幅 1.2 m:
  x=4.4 の壁は y -1.0〜0.2 / 3.0〜4.2、y=1.6 の壁は x 1.8〜3.0 / 5.8〜7.0
- 外から入る入口は西壁の 1 箇所 (x=0.4, y -0.6〜0.6)。ロボットはその正面 0.4 m に立つ

```
y=+5.6 ┌───────────┬─┬───────────┐
       │ Room NW    開  Room NE   │
       │ 探索1      口  探索2     │
y=+1.6 ├───開口──┼─┼──開口───┤
       │ Room SW    開  Room SE   │
       │ 片付け1    口  片付け2   │
y=-2.4 └───────────┴─┴───────────┘
      x=0.4       x=4.4        x=8.4
```

| 部屋 | 役割 | 家具 (models/wrs_models) |
|---|---|---|
| Room SW (西下) | 片付け 1 | 段違い棚 `wrc_stair_like_drawer` + 引き出し `trofast_1..3`、長机 `wrc_long_table_0` + トレイ 2 + コンテナ 2 |
| Room SE (東下) | 片付け 2 | ビン `wrc_bin_black` / `wrc_bin_green`、高い机 `wrc_tall_table`、本棚 `wrc_bookshelf_2` (pick 用) |
| Room NW (西上) | 探索 1 | 本棚 `wrc_bookshelf` (place 用、見本を置く) + ソファ (`furniture:`) + 床の散乱物体 |
| Room NE (東上) | 探索 2 | 長机 `wrc_long_table` + 緑ビン `wrc_bin_green_2` + 丸テーブル・椅子 (`furniture:`) + 床の散乱物体 |

物を置ける面 (placement.yaml の `placements:` で使う家具名と段 `tier`):

| 家具名 | 天板の高さ |
|---|---|
| `wrc_long_table` / `wrc_long_table_0` | 0.40 m (1 段) |
| `wrc_tall_table` | 0.60 m (1 段) |
| `wrc_bookshelf` / `wrc_bookshelf_2` | tier 0=0.06 / 1=0.50 / 2=0.80 / 3=1.05 / 4=2.02 (tier 4 は HSR には高すぎる) |
| `round_table` (`furniture:` の丸テーブル) | 0.49 x scale。Seed ファイルにも同じ `furniture:` を書いて天板を計算させる |

### 引き出し (Room SW)

段違い棚のオレンジの箱 (`trofast_1/2/3`) は、起動時に棚との間へ直動ジョイント
(`PrismaticJoint`) が張られ、取っ手を掴んで引ける引き出しになる。手前方向にだけ
0.30 m スライドし、引ききると止まり、抜け落ちない。段違い棚の無い world では何も起きない。

| 環境変数 | 既定 | 説明 |
|---|---|---|
| `DRAWER_JOINTS` | 1 | 0 で引き出し化をやめ、従来の「棚に置いてあるだけの箱」に戻す |
| `DRAWER_PULL_LIMIT` | 0.30 | 引き出せる距離 [m] |
| `DRAWER_DAMPING` | 15.0 | 引き心地 (粘性抵抗)。重ければ下げる、勢い余るなら上げる |
| `DRAWER_DAMPING_MAX_FORCE` | 200.0 | 粘性抵抗が出せる力の上限 [N] |

起動ログに `[drawer] trofast_1 -> prismatic joint (0..0.30 m, damping=15.0)` が 3 行出ていれば設定できている。

### 未知物体 (models/unknown_objects)

競技当日に追加される未知物体 (`logitech_m310_mouse`, `coca_cola_bottle`, `hamburger`, `dualshock`) は
`models/unknown_objects/<名前>/model.usd` に置いてあり、YCB と同じように `object:` にフォルダ名を
書くだけで出せる。追加のしかたは [`models/unknown_objects/README.md`](../../../../models/unknown_objects/README.md)。

## 競技モード

`task.yaml` の `time_limit` (秒, シミュレータ内時間) が 0 より大きいと競技モードになる。
`make up TASK=tidyup TIME=<秒>` の `TIME` (環境変数 `TASK_TIME`) で上書きでき、`TIME=0` で無効化。

- `arena_cameras: true` のとき、部屋の外側の高い位置に観戦カメラ 4 台 (北/南/東/西) を作り、
  4 画面を 2x2 に合成した動画をシミュレータ内部で直接 mp4 に録画する (ROS トピックは使わない)。
  動画は 1/fps シミュレータ秒 = 1 フレームなので、描画が実時間より遅くても
  **動画の再生時間 = シミュレータ内の競技時間** になる。
- 競技時間が来たら動画を保存してシミュレータが自動終了する。
- 保存先はコンテナ内 `/recordings/<日付_時刻>/arena.mp4` (ホストのリポジトリ直下 `recordings/` に
  bind mount。git 管理外)。実行のたびに新しいフォルダが作られるので上書きされない。

```
┌─────────┬─────────┐
│  NORTH  │  EAST   │   各画面の左下にカメラ名、
├─────────┼─────────┤   全体の左上に「TASK 経過時間 / 競技時間」を焼き込む
│  SOUTH  │  WEST   │
└─────────┴─────────┘
```

カメラの位置・画角・解像度・fps は `placement.yaml` の `arena_cameras:` で指定する
(書き方は同ファイルのコメントと `hsr_omniverse_sim/task/arena_cameras.py` 冒頭を参照)。無ければ床の中心から自動配置。

## 固定配置 (Seed)

競技では物体の配置を Seed 1〜4 の 4 通りに固定している。`make up TASK=tidyup SEED=<n>`
(環境変数 `PLACEMENT_SEED=<n>`) を付けると、`task.yaml` の `placement_seeds: placement.seed{n}.yaml`
に従って `placement.seed<n>.yaml` が読まれ、**`objects:` (床・家具上・障害物) だけ**がそのファイルの
内容になる。`robot:` / `people:` / `furniture:` の実配置 / `arena_cameras:` は `placement.yaml` のまま。
ファイルが無い Seed 番号や、`TASK` 無しの `SEED=` はエラーで止まる。

各 Seed の内訳: 得点対象 22 個 (Food 5 + Shape 5 + Kitchen 5 + Task 5 + Unknown 2) を
床 14 + 家具上 8 に配置し、別枠で place 用棚のカテゴリ見本 2 個と、通路上の障害物
(Coffee can か Toy airplane) 1 個を置く。カテゴリとモデル名の対応、共通のスロット座標は
`object_list.yaml` を正本とし、新しい Seed を作るときはそこから物体名を埋める。

## 観戦カメラの調整 (tune)

```bash
make tune TASK=tidyup          # カメラ 4 台だけ作って起動 (録画も自動終了もしない)
make tune-apply TASK=tidyup    # 調整結果を hsr_omniverse_sim/config/tasks/tidyup/placement.yaml に反映
```

1. Isaac Sim の Viewport 左上のカメラ選択 (Perspective と書いてある所) から `Camera_north` などを
   選ぶと、そのカメラの見え方がそのまま映る。
2. その状態でマウス/WASD で視点を動かすと、カメラ自体が動く。Stage ツリーで
   `/World/ArenaCameras/Camera_north` を選び、Property パネルの Translate に数値を打ち込んでもよい。
3. 画角は Property > Camera > Focal Length で変えられる (小さいほど広角)。
4. 動かすたびに次の 2 つが書き出される:
   - `recordings/tune/arena_cameras.yaml` … placement.yaml に貼る `arena_cameras:` ブロック
   - `recordings/tune/preview.png` … 実際に録画される 2x2 の絵 (最終確認はこちらで)
5. `make tune-apply TASK=tidyup` で `arena_cameras:` ブロックだけが `placement.yaml` に
   差し替わる (他のセクションとコメントは残る。元ファイルは `placement.yaml.bak` に残る)。
   `make tune` だけでは反映されない (コンテナからは hsr_omniverse_sim/config/ が読み取り専用のため)。

注意: カメラを横に傾けた (roll) 状態は position/look_at 形式では保存できず、反映すると水平に戻る
(起動ログに WARNING が出る)。

## reset_world

`/isaac/reset_world` サービスのほか、トピック 1 発でもリセットできる:

```bash
ros2 topic pub --once /isaac/reset_world_request std_msgs/msg/Empty
```

物体・引き出し・トレイなどの動く剛体は初期姿勢へ、GUI (ギズモ) で動かした家具・壁・人も起動時の
姿勢へ戻る。モデル内部のメッシュまで戻すために起動時にサブツリー全体の姿勢を控えている。
起動が重いときだけ `RESET_DEEP_CAPTURE=0` で root だけの旧挙動に戻せる
(実測値は起動ログの `[reset_world] captured ...`)。
