# configs/textures/

[`scene_dressing`](../../scene_dressing/) で参照されるテクスチャ画像。
リポジトリに同梱されている**サンプル/プレースホルダ用の小さな画像**で、
`git clone → make ros2 build → make ros2 up` で何も用意せずに動作確認できる。

合計サイズは 100KB 未満。本番用の高品質テクスチャはリポジトリに入れない設計。

## 中身

| ファイル | 用途 | 色 | サイズ |
|---|---|---|---|
| `floor.jpg` | 床 | グレータイル模様 | 256×256 |
| `wall_1.jpg` | 北壁 (+Y) | 青 + "NORTH" ラベル | 512×256 |
| `wall_2.jpg` | 南壁 (-Y) | 赤 + "SOUTH" ラベル | 512×256 |
| `wall_3.jpg` | 東壁 (+X) | 緑 + "EAST" ラベル | 512×256 |
| `wall_4.jpg` | 西壁 (-X) | 黄 + "WEST" ラベル | 512×256 |

各画像には "PLACEHOLDER" の文字と方角ラベルが入っているので、シーン内で壁の向きを目視確認できる。

## コンテナ内マッピング

| ホスト | コンテナ |
|---|---|
| `configs/textures/` | `/data/textures/` |

[`../dressing.yaml`](../dressing.yaml) からは `/data/textures/floor.jpg` などコンテナ内パスで参照される。

bind mount (compose) とイメージ焼き込み (Dockerfile) の両方で設定されているので:
- **mount あり**: ホスト側 `configs/textures/` の編集が即反映 (build 不要)
- **mount なし**: イメージに焼き込まれた版が使われる (完全自己完結)

## 本番テクスチャを使うには

3 通り。お好みの方法で:

### 方法 A: ファイルを上書きする(最も簡単)

`configs/textures/` 内の各 `.jpg` を本物のテクスチャで上書きする。
ファイル名はそのまま (`floor.jpg`, `wall_1.jpg`, ... `wall_4.jpg`)。

git 管理を避けたい場合は `.gitignore` に `configs/textures/*.jpg` を追加してから上書きする。

### 方法 B: compose のマウント先を変える

[`env_docker/docker-compose-ros2.yml`](../../env_docker/docker-compose-ros2.yml) の以下を別ディレクトリに切り替える:

```yaml
- ../configs/textures:/data/textures:ro
```
↓
```yaml
- ${HOME}/datasets/SceneTextures:/data/textures:ro
```

これで `~/datasets/SceneTextures/` 内の画像が使われる(repo はノータッチ)。

### 方法 C: dressing.yaml で別パスを指定

[`../dressing.yaml`](../dressing.yaml) の `dressing_presets.lab.floor_texture` などに直接別パスを書く方法。新しいディレクトリを compose で `/data/<NAME>/` にマウントしてから、dressing.yaml でそのパスを指定する。

新テーマ(例: `office`)を増やすときはこの方法が自然 ([`../README.md`](../README.md) 参照)。

## なぜ configs/ に置いてあるか

- **設定とデータが近くにある** — dressing.yaml と同じディレクトリで管理されているので「ここを編集すれば見た目が変わる」がわかりやすい
- **新規セットアップの摩擦を減らす** — `git clone` した直後から動く
- **Docker image にも焼き込まれている** — マウントなしでも動く
- **想定するファイル形式・命名規則を提示する** — 本番テクスチャをどう用意すれば良いか一目で分かる
