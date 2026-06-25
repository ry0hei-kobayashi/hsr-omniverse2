#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
furniture_spawn: placement.yaml の furniture: セクションに従って「家具(机・椅子など)」を
                 本物のメッシュ USD として配置する。

なぜ作るか:
  これまで机などの家具は worlds/*.world の中で unit_box (ただの箱) を拡大して表現して
  いた。箱なので見た目がレストランらしくない。一方、人 (people_spawn.py) は Isaac 公式
  アセットサーバ (ネット) から本物の人体 USD を取得して置いている。
  同じサーバには「机・椅子」など本物の家具 USD も入っているので、人と同じ仕組みで
  本物の家具をネットから取得して任意の位置に置けるようにする。これが本モジュール。

  ※ world の <include> 経由でも本物 USD は読めるが、それは usd/wrs_models/ にある
    ローカルファイル限定で、アセットサーバ上の家具 URL は指定できない。さらに向き
    (upAxis) や単位 (metersPerUnit) の補正もされない。本モジュールはそれらを自動補正
    し、設定 YAML から URL と位置を指定できるようにする (人と同じ思想)。

配置のしかた (configs/tasks/<task>/placement.yaml):
  furniture:
    - usd: /NVIDIA/Assets/ArchVis/Residential/Furniture/DiningSets/DesPeres/DesPeres_Table.usd
      name: table_1          # シーン内の名前 (重複しても自動で連番にする)
      x: 3.0                 # world 座標 (m)
      y: 1.3
      yaw: 0                 # 向き (度)。0=+X, 90=+Y, 180=-X, 270=-Y
      # z: 0.0               # 床の高さ (省略時 0。snap_to_floor で底を床に合わせる)
      # scale: 1.0           # 追加の拡大率 (省略時 1.0)。単位 (cm/m) は自動補正されるので
      #                        基本は触らなくてよい。少し大きく/小さくしたいときだけ使う。
      # collision: true      # 当たり判定を付けるか (省略時 true)
      # collision_approximation: convexHull   # none/convexHull/convexDecomposition/boundingCube
      # snap_to_floor: true  # 底面を床 (z) にぴったり合わせるか (省略時 true)

usd の書き方 (3 通り):
  (A) "/" で始まる        -> アセットサーバ上のパス。assets_root + usd で取得 (ネット)。
      例: /NVIDIA/Assets/ArchVis/... や /Isaac/Props/...
  (B) "omniverse://" や "http(s)://" で始まる -> その URL をそのまま使う。
  (C) それ以外            -> リポジトリ内 usd/ 配下のローカルファイルとして探す。
      例: my_models/my_table/model.usd

自動補正 (重要):
  本物の家具 USD は作られ方がまちまちで、そのまま置くと「横倒し」「100 倍デカい」
  「床に埋まる/浮く」ことがある。本モジュールは USD のメタデータを読んで次を自動補正する。
    - upAxis=Y で作られていれば X 軸 +90 度回して立たせる (Z-up はそのまま)。
    - metersPerUnit (cm/m の別) を読んで、メートル世界に合うよう一律スケールする
      (例: cm 単位 = metersPerUnit 0.01 のモデルは 0.01 倍して実寸にする)。
    - snap_to_floor: 配置後にバウンディングボックスを測り、底面が床 (z) に来るよう上下に
      ずらす (原点が中心/床のどちらでも床にぴったり乗る)。

公開:
  - spawn_furniture(assets_root, kit, config_path=None): 家具を配置し、配置数を返す。
  - load_config(path): YAML を辞書として読む (デバッグ用)。
"""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

import yaml

from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.usd
from omni.physx.scripts import utils as physx_utils
from pxr import Gf, Usd, UsdGeom


# ============================================================
# 設定ファイルの場所 (object_placement.py / people_spawn.py と同じ既定)
# ============================================================
DEFAULT_CONFIG_PATH: str = "/app/configs/placement.yaml"
CONFIG_PATH: str = os.environ.get("FURNITURE_CONFIG", DEFAULT_CONFIG_PATH)

# 家具をまとめて置く親 prim (GUI で見やすいようまとめる。人の /World/People と同じ思想)。
FURNITURE_ROOT = "/World/Furniture"

# 有効な当たり判定の近似形状 (PhysX)。
_VALID_APPROX = {
    "none",                 # 三角メッシュそのまま (静止物むけ・正確だが重い)
    "convexHull",           # 凸包 1 つ (軽くて十分。家具の既定)
    "convexDecomposition",  # 凹みも表現 (重いが最も正確)
    "boundingCube",         # 直方体 (一番軽い)
    "boundingSphere",       # 球
    "meshSimplification",   # 簡略化メッシュ
}
DEFAULT_APPROX = "convexHull"


def log(message: str) -> None:
    print(f"[furniture] {message}")


# ============================================================
# オフライン用ローカルミラーの解決 (会場=WiFiなし対策)
# ============================================================
# people_spawn.py と同じ仕組み。usd: が "/" 始まり(アセットサーバ上のパス)のとき、
# まず usd/isaac_offline/ に同じ構成で同梱されていないかを見て、あればローカルから
# 読む(ネット不要)。無ければ従来どおり assets_root(オンライン)に連結する。
_MIRROR_DIRS = [
    "/app/usd/isaac_offline",  # コンテナ内 (usd マウント / イメージ ADD で配備済み)
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                 "usd", "isaac_offline"),  # ホストで直接実行したとき
]
if os.environ.get("ISAAC_OFFLINE_MIRROR"):
    _MIRROR_DIRS.insert(0, os.environ["ISAAC_OFFLINE_MIRROR"])


def _resolve_server_path(server_path: str, assets_root: str) -> str:
    """アセットサーバ上のパス("/Isaac/...", "/NVIDIA/...") をローカルミラー優先で解決。

    ローカルミラー(usd/isaac_offline/)に同じ構成のファイルがあればその実ファイル
    パスを返し(ネット不要)、無ければ従来どおり assets_root に連結した URL を返す。
    """
    rel = server_path.lstrip("/")
    for base in _MIRROR_DIRS:
        cand = os.path.join(base, rel)
        if os.path.exists(cand):
            return cand
    return f"{assets_root.rstrip('/')}{server_path}"


# ============================================================
# YAML 読み込み (object_placement.py と同じフォールバック)
# ============================================================
def load_config(path: str) -> Dict[str, Any]:
    """placement.yaml を辞書として読み込む (家具の配置は furniture: セクション)。"""
    if not os.path.exists(path):
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        fallback = os.path.join(repo_root, "configs", "placement.yaml")
        if os.path.exists(fallback):
            path = fallback
        else:
            log(f"WARNING: config not found: {path} (nor {fallback})")
            return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    log(f"loaded config: {path}")
    return data


# ============================================================
# 1 エントリの正規化
# ============================================================
def _parse_item(item: Any, idx: int) -> Optional[Dict[str, Any]]:
    """YAML の 1 エントリを正規化する。不正なら None。"""
    if not isinstance(item, dict):
        log(f"WARNING: furniture[{idx}] は辞書ではありません: {item!r}。スキップ。")
        return None
    usd = item.get("usd")
    if not usd:
        log(f"WARNING: furniture[{idx}] に usd: がありません。スキップ。")
        return None
    name = str(item.get("name", f"furniture_{idx}"))
    try:
        x = float(item.get("x", 0.0))
        y = float(item.get("y", 0.0))
        z = float(item.get("z", 0.0))
        yaw_deg = float(item.get("yaw", 0.0))
        scale = float(item.get("scale", 1.0))
    except (TypeError, ValueError):
        log(f'WARNING: furniture "{name}" の数値が読めません: {item!r}。'
            f"スキップ (小数点は '.' で書いてください)。")
        return None
    approx = str(item.get("collision_approximation", DEFAULT_APPROX))
    if approx not in _VALID_APPROX:
        log(f'WARNING: furniture "{name}" の collision_approximation="{approx}" は'
            f"不明です。{DEFAULT_APPROX} を使います。使える値: {sorted(_VALID_APPROX)}")
        approx = DEFAULT_APPROX
    return {
        "usd": str(usd),
        "name": name,
        "x": x, "y": y, "z": z,
        "yaw_deg": yaw_deg,
        "scale": scale,
        "collision": bool(item.get("collision", True)),
        "approx": approx,
        "snap_to_floor": bool(item.get("snap_to_floor", True)),
    }


def _resolve_usd_url(usd: str, assets_root: str) -> Optional[str]:
    """usd 指定を実際に開ける URL/パスに変換する (書き方 A/B/C は本ファイル冒頭参照)。"""
    # (B) すでに URL なら、そのまま。
    if usd.startswith(("omniverse://", "http://", "https://")):
        return usd
    # (A) "/" 始まりはアセットサーバ上のパス。assets_root に連結。
    #     assets_root は get_assets_root_path() の戻り値 (例: .../Assets/Isaac/4.5)。
    #     people_spawn.py が assets_root + "/Isaac/People/..." で人を取得しているのと
    #     同じ root で、家具は assets_root + "/NVIDIA/Assets/ArchVis/..." を使う
    #     (どちらも同じ root の隣り合うフォルダ。Isaac 標準コードの慣用)。
    #     末尾スラッシュの有無で "//" にならないよう rstrip しておく。
    #     さらに、ローカルミラー(usd/isaac_offline/)に同梱があればそちらを優先し、
    #     会場(ネットなし)でもサーバ家具を配置できるようにする(無ければオンライン)。
    if usd.startswith("/"):
        return _resolve_server_path(usd, assets_root)
    # (C) それ以外はリポジトリ内 usd/ 配下のローカルファイルを探す。
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidates = [
        os.path.join("/app/usd", usd),
        os.path.join(repo_root, "usd", usd),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    log(f"WARNING: ローカル USD が見つかりません: {usd} (探した場所: {candidates})")
    return None


def _asset_exists(url: str) -> bool:
    """解決した URL/パスが実在するかを参照前に確認する。

    なぜ必要か: add_reference_to_stage は参照先が存在しなくても「空の prim」を
    作ってしまい、prim.IsValid() が True になる。そのため URL を間違えても
    「黙って何も置かない (空配置)」状態になり、原因が分かりにくい。
    参照する前にここで存在を確かめ、無ければ明示的に警告してスキップできるようにする。
    """
    # ローカルファイルはそのまま存在確認。
    if not url.startswith(("omniverse://", "http://", "https://")):
        return os.path.exists(url)
    # リモートは omni.client で確認 (list_people_anims.py と同じ omni.client)。
    try:
        import omni.client
        result, _ = omni.client.stat(url)
        return result == omni.client.Result.OK
    except Exception as ex:
        # 確認手段が無い等のときは従来どおり「存在する前提」で進める (壊さない)。
        log(f"NOTE: 存在確認をスキップ ({url}): {ex!r}")
        return True


def _read_up_and_units(url: str) -> tuple:
    """参照元 USD の (upAxis が Y か?, metersPerUnit) を返す。

    開けなければ「Z-up・metersPerUnit=1.0」を仮定して警告する (= 補正なし)。
    Isaac の中では Usd.Stage.Open はアセットサーバ URL も解決できる
    (drop_object がローカル USD で使っているのと同じ API)。
    """
    try:
        src = Usd.Stage.Open(url)
        if src is None:
            raise RuntimeError("stage is None")
        is_y_up = UsdGeom.GetStageUpAxis(src) == UsdGeom.Tokens.y
        mpu = UsdGeom.GetStageMetersPerUnit(src) or 1.0
        return is_y_up, float(mpu)
    except Exception as ex:
        log(f"WARNING: USD メタデータを読めませんでした ({url}): {ex!r}。"
            f"Z-up・等倍を仮定します。大きさ/向きが変なら scale/ で手動調整してください。")
        return False, 1.0


# ============================================================
# 1 体ぶんの配置
# ============================================================
def _spawn_one(stage: Usd.Stage, assets_root: str, item: Dict[str, Any],
               kit: Any) -> bool:
    """家具を 1 つ配置する。成功で True。"""
    name = item["name"]
    prim_path = f"{FURNITURE_ROOT}/{name}"

    url = _resolve_usd_url(item["usd"], assets_root)
    if url is None:
        return False

    # 参照前に実在確認する。誤ったパスを「空の prim」で黙って通さないため。
    if not _asset_exists(url):
        log(f'WARNING: furniture "{name}" の USD が見つかりません '
            f"(パス誤り/未配備の可能性): {url}。スキップします。")
        return False

    # 1) USD を参照として読み込む (ネット or ローカル)。参照解決のため数回更新。
    add_reference_to_stage(url, prim_path)
    kit.update()
    kit.update()

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        log(f'WARNING: furniture "{name}" の読み込みに失敗: {url}')
        return False

    # 2) 向き・単位を読み、補正スケールを決める。
    #    stage は 1.0 m/unit なので、cm 単位 (mpu=0.01) のモデルは 0.01 倍で実寸になる。
    is_y_up, mpu = _read_up_and_units(url)
    scale_factor = mpu * item["scale"]

    # 3) 位置・向き・スケールを設定する。
    #    USD の xform op は「最後に足したものが最初にジオメトリへ効く」。
    #    効かせたい順 = スケール → (Y-up なら X 回転で立たせる) → Z 回転(向き) → 平行移動。
    #    そのため足す順は 平行移動 → Z 回転 → X 回転 → スケール の逆順で書く。
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    translate_op = xf.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(item["x"], item["y"], item["z"]))
    xf.AddRotateZOp().Set(item["yaw_deg"])     # 向き (度)
    if is_y_up:
        xf.AddRotateXOp().Set(90.0)            # Y-up -> Z-up (Z-up モデルでは付けない)
    xf.AddScaleOp().Set(Gf.Vec3d(scale_factor, scale_factor, scale_factor))
    kit.update()

    # 4) snap_to_floor: 配置後の実際の底面を測り、底が z に来るよう上下にずらす。
    #    (原点が床/中心どちらのモデルでも、ちゃんと床に乗る。)
    if item["snap_to_floor"]:
        try:
            bbox_cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                useExtentsHint=True,
            )
            rng = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if not rng.IsEmpty():
                bottom_z = rng.GetMin()[2]
                shift = item["z"] - bottom_z      # 底を z に合わせるための上下ずらし量
                translate_op.Set(Gf.Vec3d(item["x"], item["y"], item["z"] + shift))
                kit.update()
            else:
                log(f'WARNING: furniture "{name}" のバウンディングボックスが空でした。'
                    f"snap_to_floor をスキップします。")
        except Exception as ex:
            log(f'WARNING: furniture "{name}" の snap_to_floor に失敗: {ex!r}。続行します。')

    # 5) 当たり判定 (collider) を付ける。剛体 (RigidBodyAPI) は付けないので
    #    「動かない固い家具」になる (world の壁/箱と同じ静的 collider)。
    if item["collision"]:
        _apply_collider_subtree(prim, item["approx"], name)

    log(f'spawned "{name}" at ({item["x"]:.2f}, {item["y"]:.2f}, {item["z"]:.2f}) '
        f'yaw={item["yaw_deg"]:.0f}deg scale={scale_factor:.4f} '
        f'(y_up={is_y_up}, mpu={mpu:.3g}) collision={item["collision"]} from {url}')
    return True


def _apply_collider_subtree(root_prim: Usd.Prim, approx: str, name: str) -> None:
    """root_prim 配下の全メッシュに静的 collider を付ける。

    setCollider は 1 つの geometry prim 向けなので、家具のように複数メッシュを
    束ねた Xform では、子孫の Mesh を辿って 1 つずつ付ける (関数名の有無に依存しない確実な方法)。
    """
    applied = 0
    for p in Usd.PrimRange(root_prim):
        if not UsdGeom.Mesh(p):
            continue
        try:
            physx_utils.setCollider(p, approximationShape=approx)
            applied += 1
        except Exception as ex:
            log(f'WARNING: furniture "{name}" の {p.GetPath()} に collider を'
                f"付けられませんでした: {ex!r}")
    if applied == 0:
        log(f'WARNING: furniture "{name}" に collider 対象のメッシュが見つかりませんでした。')


# ============================================================
# 公開関数
# ============================================================
def spawn_furniture(assets_root: str, kit: Any,
                    config_path: str | None = None) -> int:
    """placement.yaml の furniture: セクションに従って家具を配置する。

    戻り値は配置できた家具の数。furniture: が無い/空のときは 0 (何も置かない)。
    furniture: が無いタスク (hri/gpsr 等) でも安全に no-op になるので、launch から
    無条件に呼んでよい。
    """
    path = config_path or CONFIG_PATH
    cfg = load_config(path)

    # furniture: は「リストそのまま」でも「{list: [...]}」でも書けるようにする。
    section = cfg.get("furniture")
    if isinstance(section, dict):
        items = section.get("list") or []
    else:
        items = section or []

    if not items:
        log("furniture が空です。家具は配置しません (既存タスクに影響なし)。")
        return 0

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(FURNITURE_ROOT).IsValid():
        UsdGeom.Xform.Define(stage, FURNITURE_ROOT)

    requested = 0
    placed = 0
    used_names: set = set()
    for idx, raw in enumerate(items):
        parsed = _parse_item(raw, idx)
        if parsed is None:
            continue
        # 名前の重複は prim パス衝突になるので連番を足して避ける (people_spawn と同じ)。
        base = parsed["name"]
        uniq = base
        n = 1
        while uniq in used_names:
            uniq = f"{base}_{n}"
            n += 1
        parsed["name"] = uniq
        used_names.add(uniq)

        requested += 1
        # 1 つの家具の失敗 (ネット取得失敗・USD 不正など) で起動全体を巻き込まないよう、
        # ここで例外を受け止めて次の家具へ進む (people_spawn と同じ「壊れても続行」方針)。
        try:
            if _spawn_one(stage, assets_root, parsed, kit):
                placed += 1
        except Exception as ex:
            log(f'WARNING: furniture "{parsed["name"]}" の配置中に例外: {ex!r}。スキップして続行します。')

    log(f"placed {placed}/{requested} furniture from {path}")
    return placed
