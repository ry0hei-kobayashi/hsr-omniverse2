# configs/textures/

[`scene_dressing`](../../scene_dressing/) で参照されるテクスチャ画像。
**本物の lab テクスチャ**をリポジトリに同梱しているので、
`git clone → make ros2 build → make ros2 up` で何も用意せず本番見た目で動く
(dev/非dev どちらでも、ホスト側のデータセット準備は不要)。

合計サイズは約 6MB (壁画像が各 1〜2MB)。

## 中身

| ファイル | 用途 |
|---|---|
| `floor.jpg` | 床 |
| `wall_1.jpg` | 北壁 (+Y) |
| `wall_2.jpg` | 南壁 (-Y) |
| `wall_3.jpg` | 東壁 (+X) |
| `wall_4.jpg` | 西壁 (-X) |
| `restaurant_floor.jpg` | レストラン風の木製フローリング床 (`restaurant` プリセット用)。出典: [Polyhaven](https://polyhaven.com/a/wood_floor) **CC0** (パブリックドメイン) |

`floor.jpg` / `wall_*.jpg` は [`../dressing.yaml`](../dressing.yaml) の `lab` プリセット (デフォルト) が、
`restaurant_floor.jpg` は `restaurant` プリセット (restaurant タスク) が参照する。

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
