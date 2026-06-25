# usd/isaac_offline/ — オフライン用 Isaac アセットのローカルミラー

会場（Wi-Fi なし）で完全オフライン動作させるために、NVIDIA Isaac Sim の
**アセットサーバ上にある人（キャラ + アニメ）一式を、サーバと同じフォルダ構成の
まま**このフォルダにコピー（ミラー）してあります。

実行時、`scripts/people_spawn.py` と `scripts/furniture_spawn.py` は

1. まずこのフォルダ（`usd/isaac_offline/<サーバと同じパス>`）にファイルがあればそれを使う（ネット不要）
2. 無ければ従来どおりアセットサーバ（オンライン）から取得

の順で解決します。詳しくはリポジトリ直下 `README.md` の「オフライン対応」を参照。

## 中身

```
Isaac/People/
├── Characters/
│   ├── Biped_Setup.usd                  人体モデルの入口（スケルトン + アニメ参照）
│   └── biped_demo/
│       ├── biped_demo_meters.usd        実体のメッシュ・マテリアル
│       └── Textures/*.png               肌のテクスチャ(diffuse/normal/roughness/metallic)
└── Animations/
    └── *.skelanim.usd                   立つ/手を振る/座る/歩く 等の「骨の動き」データ
```

- アニメ（`*.skelanim.usd`）は「関節を時間ごとにどの角度にするか」という数値データだけの
  ファイルです。一度ローカルに読み込めば、**再生中（手を振る等）にネットは要りません**。
- `Biped_Setup.usd` は内部で `OmniPBR.mdl`（コア材質）を参照しますが、これは Isaac Sim 本体に
  内蔵されているためミラー不要です（オフラインでも解決されます）。

## 作り直す・増やす（オンライン環境で一度だけ）

```bash
make ros2 offline-assets
# 追加したいサーバ上アセットがあれば:
make ros2 offline-assets FETCH_ARGS='--path /NVIDIA/Assets/ArchVis/.../Table.usd'
```

生成ロジックは [`scripts/fetch_isaac_offline_assets.py`](../../scripts/fetch_isaac_offline_assets.py)。
取得対象の起点一覧は同ファイルの `DEFAULT_SEEDS`（= `people_spawn.py` の `CHARACTER_USD` /
`MOTION_FILES` と一致させること）。依存ファイル（メッシュ・テクスチャ・参照アニメ）は
スクリプトが参照を再帰的にたどって自動で集めます。

## コミットについて

別 PC でも会場で人を出すには、このフォルダ（約 134MB）がその PC に存在している必要があります。
**リポジトリにコミットしておけば**クローンするだけで揃います（推奨）。コミットしない運用にする
場合は、各 PC で会場前に `make ros2 offline-assets` を実行してください。

## 出典・ライセンスの注意

ここに含まれるアセットは **NVIDIA Isaac Sim のアセットライブラリ由来**です
（`omniverse-content-production` の `Assets/Isaac/4.5/Isaac/People/`）。Isaac Sim と組み合わせて
オフライン利用するためにミラーしたものです。**このリポジトリを外部公開・再配布する場合は、
NVIDIA のアセット利用規約を確認**してください（チーム内のオフライン利用が目的です）。
机・椅子など見た目重視の家具で配布制約を避けたいものは、CC0 アセット（`usd/restaurant/`）を
使っています。
