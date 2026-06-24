# usd/restaurant/ — Restaurant タスク用の同梱家具 (CC0)

`configs/tasks/restaurant/placement.yaml` の `furniture:` が参照する、机・椅子の USD。
**ネット取得をやめてリポジトリに同梱**したもの（起動時にアセットサーバへ取りに行かない）。
読み込みは `scripts/furniture_spawn.py`。

## 中身

| パス | 内容 | 実寸 (実測) |
|---|---|---|
| `round_table/model.usd` | 円形テーブル（丸机・やや低めのカフェ風） | 直径 約1.3m / 高 0.49m |
| `long_table/model.usd` | 木製の長テーブル（ロボット右脇の長机） | 1.33 × 0.56 × 0.83 m |
| `dining_chair/model.usd` | ダイニングチェア | 0.43 × 0.58 × 0.97 m |
| `*/textures/*` | 各 USD が相対参照する PBR テクスチャ（diff=jpg, nor/rough/metal=exr, 1k） | 1024px |

- いずれも **Z-up・metersPerUnit=1.0(メートル)・底面 z=0**。`furniture_spawn.py` が向き・床着地を自動補正する
  (このCC0アセットはメートル/Z-upなのでスケール・回転補正は実質不要)。

## 出典とライセンス

- **Poly Haven (https://polyhaven.com) の CC0（パブリックドメイン）3Dモデル**。誰でも再配布・改変可。
  - `round_table`  ← `coffee_table_round_01`
  - `long_table`   ← `WoodenTable_03`
  - `dining_chair` ← `dining_chair_02`
- USD 形式（1k テクスチャ）をそのまま同梱。CC0 なので GitHub 公開リポジトリにそのまま含められる。

（床の木目テクスチャ `configs/textures/restaurant_floor.jpg` も Poly Haven CC0。）

## 備考

- 円形テーブルは CC0 にダイニング高さ（〜0.75m）のものが無く、`coffee_table_round_01`（高 0.49m）を
  採用しているため、ダイニングチェアと合わせると少し低めに見える。気になる場合は
  `placement.yaml` の `round_table` を別のテーブル（例: `long_table` と同型の四角いダイニング高テーブル）に
  差し替えるか、Poly Haven の他の CC0 テーブルを同じ手順で同梱して差し替える。
