#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
people_spawn: configs/placement.yaml の people: セクションに従って「人」を配置する。

object_placement.py (物体配置) と同じ作りで、YAML を読んで人を置くモジュール。
launch_isaacsim.py から spawn_people(...) を呼ぶ。

アニメーションの仕組み (重要):
  Isaac Sim 4.5.0 / omni.anim.people では AnimationGraph をスクリプトから
  正しく動かせなかったため、Biped_Setup.usd を使い UsdSkel に直接
  アニメーションクリップをバインドする方式を採る (妥協案)。
    - 人モデルは Isaac 公式アセットサーバ (ネット) から取得する。
    - Biped は Y-up で作られているので Z-up ステージ用に X 軸 +90 度回す。
    - アニメは「メインループで kit.update() を回さないと評価されない」点に注意。
      → launch_isaacsim.py 側のループに kit.update() を入れること。

公開:
  - spawn_people(assets_root, kit, config_path=None): 人を配置し、配置数を返す。
  - load_config(path): YAML を辞書として読む (デバッグ用)。
"""
from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Optional

import yaml

from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel


# ============================================================
# 設定ファイルの場所
# ============================================================
# デフォルトは /app/configs/placement.yaml (docker-compose で bind mount)。
# robot/objects/people をまとめた設定ファイルで、人の配置は people: セクション。
# 環境変数 PEOPLE_CONFIG で上書き可能。
DEFAULT_CONFIG_PATH: str = "/app/configs/placement.yaml"
CONFIG_PATH: str = os.environ.get("PEOPLE_CONFIG", DEFAULT_CONFIG_PATH)

# 人モデルとアニメーションの置き場所 (Isaac アセットサーバ内の相対パス)。
# assets_root は呼び出し側 (launch_isaacsim.py) が get_assets_root_path() で得たもの。
CHARACTER_USD = "/Isaac/People/Characters/Biped_Setup.usd"
ANIM_DIR = "/Isaac/People/Animations"

# 人を置く prim の親 (まとめておくと GUI で見やすい)。
PEOPLE_ROOT = "/World/People"

# Biped_Setup 内部の prim パス (復元ガイドより)。
#   SkelRoot:  アニメ評価をブロックする AnimationGraphAPI が付いている所
#   Skeleton:  実際にアニメをバインドする Skeleton prim
_SKELROOT_SUBPATH = "biped_demo_meters"
_SKELETON_SUBPATH = "biped_demo_meters/Root"
# キャラ内のアニメをまとめるスコープ (外部アニメをここに読み込む)。
_ANIM_SCOPE_SUBPATH = "CharacterAnimation/Animation"

# 使えるモーション名 -> アニメ USD ファイル名。
# (Isaac の People/Animations にあるクリップ。ネット取得。)
MOTION_FILES: Dict[str, str] = {
    "stand_idle_loop": "stand_idle_loop.skelanim.usd",
    # stand_idle_wave_loop: 立つ→手を振る→手を下ろす→立つ、を繰り返す。
    #   このライブラリには「手を上げっぱなしで振り続ける」専用クリップが無い
    #   (以前あると思っていた "waving" は実在しなかった)。手を上げたままに
    #   見せたいときは、このクリップの「手を上げて振っている区間」だけを
    #   loop_window (placement.yaml の people:) でループさせる。
    "stand_idle_wave_loop": "stand_idle_wave_loop.skelanim.usd",
    "LookAround": "LookAround.skelanim.usd",
    "Sit": "Sit.skelanim.usd",
    "stand_walk_loop": "stand_walk_loop.skelanim.usd",
    "push_button": "push_button.skelanim.usd",
    "type_keyboard": "type_keyboard.skelanim.usd",
}
DEFAULT_MOTION = "stand_idle_loop"


def log(message: str) -> None:
    print(f"[people] {message}")


# ============================================================
# YAML 読み込み
# ============================================================
def load_config(path: str) -> Dict[str, Any]:
    """placement.yaml を辞書として読み込む (人の配置は people: セクション)。

    指定パスが無ければ、このファイルから見た repo 内の configs/placement.yaml
    を探す (ホストで直接実行したときのフォールバック)。
    """
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


def _parse_person(item: Any, idx: int) -> Optional[Dict[str, Any]]:
    """YAML の 1 エントリを正規化する。不正なら None を返す。"""
    if not isinstance(item, dict):
        log(f"WARNING: people[{idx}] は辞書ではありません: {item!r}。スキップ。")
        return None
    name = str(item.get("name", f"person_{idx}"))
    try:
        x = float(item.get("x", 0.0))
        y = float(item.get("y", 0.0))
        z = float(item.get("z", 0.0))
        yaw_deg = float(item.get("yaw", 0.0))
    except (TypeError, ValueError):
        log(f'WARNING: person "{name}" の座標が数値として読めません: {item!r}。'
            f"スキップ (小数点は '.' で書いてください)。")
        return None
    motion = str(item.get("motion", DEFAULT_MOTION))
    if motion not in MOTION_FILES:
        log(f'WARNING: person "{name}" の motion="{motion}" は未知です。'
            f"{DEFAULT_MOTION} を使います。使える名前: {list(MOTION_FILES)}")
        motion = DEFAULT_MOTION
    return {"name": name, "x": x, "y": y, "z": z,
            "yaw_deg": yaw_deg, "motion": motion}


def _parse_loop_window(people_cfg: Dict[str, Any],
                       loop_duration: float) -> tuple:
    """people セクションの loop_window を読み、(開始秒, 終了秒) を返す。

    指定が無い・不正なときは (0.0, loop_duration)（＝従来どおり全体をループ）。
    start/end が逆だったり範囲外のときは警告して既定に戻す。
    end がクリップ全長 (loop_duration) を超える場合は、誰も固まらないように
    全長まで自動で切り詰める (仮の値のまま実行しても壊れないように)。
    """
    default = (0.0, loop_duration)
    win = people_cfg.get("loop_window")
    if win is None:
        return default
    if not isinstance(win, dict):
        log(f"WARNING: loop_window は start/end を持つ辞書で書いてください: "
            f"{win!r}。無視します。")
        return default
    try:
        start = float(win.get("start", 0.0))
        end = float(win.get("end", loop_duration))
    except (TypeError, ValueError):
        log(f"WARNING: loop_window の start/end が数値として読めません: {win!r}。"
            f"無視します (小数点は '.' で書いてください)。")
        return default
    if start < 0.0:
        log(f"WARNING: loop_window.start={start:.2f} が負です。0.0 にします。")
        start = 0.0
    # end はクリップ全長を超えてはいけない (超えると再生ヘッドがクリップの外に
    # 出て、最後のポーズで固まって見える)。全員が 1 本の時計を共有するので、
    # 一番短いクリップ長 (loop_duration) を上限にすれば誰も固まらない。
    # これにより、未調整の仮の end のまま実行しても安全になる。
    if loop_duration > 0.0 and end > loop_duration:
        log(f"WARNING: loop_window.end={end:.2f}s がクリップ全長 "
            f"{loop_duration:.2f}s を超えています。固まりを避けるため "
            f"{loop_duration:.2f}s に切り詰めます。")
        end = loop_duration
    if end <= start:
        log(f"WARNING: loop_window が不正です (start={start:.2f} >= "
            f"end={end:.2f})。無視して全体をループします。")
        return default
    log(f"loop_window 指定あり: {start:.2f}s..{end:.2f}s の区間だけをループします。")
    return (start, end)


# ============================================================
# 1 体ぶんの配置 + アニメーションのバインド
# ============================================================
def _strip_animation_graph_api(skel_root_prim: Usd.Prim) -> None:
    """SkelRoot から AnimationGraphAPI を外す。

    これが付いていると UsdSkel への直接バインドがブロックされてアニメが
    動かない (復元ガイドの知見)。apiSchemas から取り除き、relationship も消す。
    """
    existing = skel_root_prim.GetMetadata("apiSchemas")
    if existing is not None:
        items = [s for s in (existing.explicitItems or [])
                 if s != "AnimationGraphAPI"]
        new_op = Sdf.TokenListOp()
        new_op.explicitItems = items
        skel_root_prim.SetMetadata("apiSchemas", new_op)
    anim_rel = skel_root_prim.GetRelationship("animationGraph")
    if anim_rel:
        anim_rel.ClearTargets(True)


def _anim_duration_seconds(stage: Usd.Stage, anim_prim: Usd.Prim) -> float:
    """アニメ prim の長さ (秒) を返す。求められなければ 0.0。

    SkelAnimation の各アトリビュート (rotations/translations/scales) の
    タイムサンプルの最後の時刻を「コマ/秒 (timeCodesPerSecond)」で割って秒に直す。
    この長さを timeline.set_end_time() に渡さないとループの折り返し位置が
    分からず、アニメが最後のポーズで止まってしまう (復元ガイドの知見)。
    """
    tcps = stage.GetTimeCodesPerSecond() or 24.0
    max_tc = 0.0
    found = False
    # anim_prim 自身か、その子孫に SkelAnimation がある。両方を調べる。
    for p in [anim_prim] + list(Usd.PrimRange(anim_prim)):
        skelanim = UsdSkel.Animation(p)
        if not skelanim:
            continue
        for attr in (skelanim.GetRotationsAttr(),
                     skelanim.GetTranslationsAttr(),
                     skelanim.GetScalesAttr()):
            if attr and attr.HasValue():
                samples = attr.GetTimeSamples()
                if samples:
                    found = True
                    max_tc = max(max_tc, samples[-1])
    if not found:
        return 0.0
    return max_tc / tcps


def _bind_motion(stage: Usd.Stage, person_path: str, assets_root: str,
                 motion: str) -> float:
    """指定モーションのアニメを人の Skeleton にバインドする。

    バインドしたアニメの長さ (秒) を返す。失敗時は 0.0。
    """
    # 外部アニメを「キャラ内の Animation スコープ」に読み込む。
    # add_reference_to_stage だと Xform になり SkelAnimation と認識されないため、
    # OverridePrim + AddReference を使う (復元ガイドの知見)。
    anim_path = f"{person_path}/{_ANIM_SCOPE_SUBPATH}/{motion}"
    anim_url = f"{assets_root}{ANIM_DIR}/{MOTION_FILES[motion]}"
    anim_prim = stage.OverridePrim(anim_path)
    anim_prim.GetReferences().AddReference(anim_url)

    # Skeleton に「このアニメを再生せよ」とバインドする。
    skel_path = f"{person_path}/{_SKELETON_SUBPATH}"
    skel_prim = stage.GetPrimAtPath(skel_path)
    if not skel_prim.IsValid():
        log(f"WARNING: Skeleton prim が見つかりません: {skel_path}。"
            f"T ポーズになる可能性があります。")
        return 0.0
    binding = UsdSkel.BindingAPI.Apply(skel_prim)
    binding.GetAnimationSourceRel().SetTargets([Sdf.Path(anim_path)])

    # アニメの長さを測ってループ用に返す。
    duration = _anim_duration_seconds(stage, anim_prim)
    if duration <= 0.0:
        log(f"WARNING: motion={motion} の長さを取得できませんでした。"
            f"ループ時間の自動設定ができないため動きが止まる可能性があります。")
    return duration


def _spawn_one(stage: Usd.Stage, assets_root: str, person: Dict[str, Any],
               kit: Any) -> float:
    """人を 1 体配置してアニメをバインドする。

    成功で「アニメの長さ (秒)」、失敗で負の値 (-1.0) を返す。
    長さが取れない (が配置は成功) ときは 0.0 を返す。
    """
    name = person["name"]
    person_path = f"{PEOPLE_ROOT}/{name}"

    # 1) Biped_Setup を読み込む (ネット取得)。
    char_url = f"{assets_root}{CHARACTER_USD}"
    add_reference_to_stage(char_url, person_path)
    # 参照を解決させるため数回更新する。
    kit.update()
    kit.update()

    prim = stage.GetPrimAtPath(person_path)
    if not prim.IsValid():
        log(f'WARNING: person "{name}" の読み込みに失敗: {char_url}')
        return -1.0

    # 2) 位置・向き。Biped は Y-up なので X 軸 +90 度で Z-up に立たせる。
    #    USD の xform op は「最後に足したものが最初に適用される」ので、
    #    RotateX(90) を最後に足す = ジオメトリに最初に効かせる。
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(person["x"], person["y"], person["z"]))
    xf.AddRotateZOp().Set(person["yaw_deg"])  # 向き (度)
    xf.AddRotateXOp().Set(90.0)               # Y-up -> Z-up (必ず最後)

    # 3) AnimationGraphAPI を外す (直接バインドのブロックを解除)。
    skel_root = stage.GetPrimAtPath(f"{person_path}/{_SKELROOT_SUBPATH}")
    if skel_root.IsValid():
        _strip_animation_graph_api(skel_root)
    else:
        log(f'WARNING: SkelRoot が見つかりません: '
            f"{person_path}/{_SKELROOT_SUBPATH}")

    # 4) モーションをバインドする (アニメの長さが返る)。
    duration = _bind_motion(stage, person_path, assets_root, person["motion"])

    log(f'spawned "{name}" at ({person["x"]:.2f}, {person["y"]:.2f}) '
        f'yaw={person["yaw_deg"]:.0f}deg motion={person["motion"]} '
        f'duration={duration:.2f}s')
    return duration


# ============================================================
# 公開関数
# ============================================================
def spawn_people(assets_root: str, kit: Any,
                 config_path: str | None = None):
    """placement.yaml の people: セクションに従って人を配置する。

    戻り値は (配置できた人数, ループ開始秒, ループ終了秒) のタプル。
    人を配置しなかったときは (0, 0.0, 0.0)。

    ループ区間 (loop_window) について:
      people.loop_window: {start, end} があると、その秒数の範囲だけを
      繰り返し再生する。stand_idle_wave_loop の「手を上げて振っている区間」だけを
      ループさせ、手を下ろす部分を再生範囲から外して「上げっぱなしで振り続ける」
      ように見せるための機能。指定が無ければ start=0、end=一番短いクリップ長。
      ※ タイムラインはシーンに 1 本だけなので、この区間は全員に共通で効く。

    ループ周期について (重要な制約):
      Isaac Sim のタイムライン (再生ヘッド) はシーンに 1 本だけで、全員が
      同じ時刻を共有する。そのため人ごとにアニメの長さが違うと「各自が独立に
      ループ」はできず、どこか 1 つの周期に全員を合わせるしかない。
        - 一番"長い"クリップに合わせると、短い人は動き終わってから折り返し
          までの間フリーズする (最後のポーズで固まる)。← これは不自然で目立つ
        - 一番"短い"クリップに合わせると、長い人は末尾が切れて途中で巻き戻る
          (小さなカクつきは出るが) 誰もフリーズしない。
      フリーズを避けたいので後者を採用し、「一番短いクリップ」を周期にする。
      「ずっと同じ動作」を繰り返すクリップなら、末尾が切れても自然に見える。
      なお loop_window を指定した場合は、この既定の周期ではなくその区間を使う。
    """
    path = config_path or CONFIG_PATH
    cfg = load_config(path)
    # placement.yaml は robot / objects / people の 3 セクション構成。
    # 人の配置は people: セクションの中の list:、再生区間は loop_window:。
    people_cfg = cfg.get("people") or {}
    people = people_cfg.get("list") or []

    if not people:
        log("people.list が空です。人は配置しません (既存動作のまま)。")
        return 0, 0.0, 0.0

    stage = omni.usd.get_context().get_stage()
    # まとめ親 (なければ作る)。
    if not stage.GetPrimAtPath(PEOPLE_ROOT).IsValid():
        UsdGeom.Xform.Define(stage, PEOPLE_ROOT)

    requested = 0
    spawned = 0
    # (人の名前, アニメ秒数) を集める。周期決定と警告に使う。
    durations: List[tuple] = []
    used_names: set = set()
    for idx, raw in enumerate(people):
        person = _parse_person(raw, idx)
        if person is None:
            continue
        # 名前の重複は prim パス衝突になるので連番を足して避ける。
        base = person["name"]
        uniq = base
        n = 1
        while uniq in used_names:
            uniq = f"{base}_{n}"
            n += 1
        person["name"] = uniq
        used_names.add(uniq)

        requested += 1
        duration = _spawn_one(stage, assets_root, person, kit)
        if duration >= 0.0:   # -1.0 は配置失敗。0.0 以上は配置成功。
            spawned += 1
            durations.append((uniq, duration))

    # ループ周期 = 長さが取れた人の中で「一番短い」秒数 (フリーズ回避)。
    valid = [(name, d) for name, d in durations if d > 0.0]
    loop_duration = min((d for _, d in valid), default=0.0)

    # 長さがバラバラなら、長い人は末尾が切れる旨を一度だけ警告する。
    if valid:
        longest_name, longest = max(valid, key=lambda nd: nd[1])
        if longest > loop_duration + 1e-3:
            log(f'INFO: アニメの長さが人により違います。フリーズを避けるため周期を'
                f'一番短い {loop_duration:.2f}s に合わせます。長い人 (例 '
                f'"{longest_name}"={longest:.2f}s) は末尾が切れて折り返します '
                f'(振り続ける系の動作なら自然に見えます)。')

    # 再生区間 (loop_window) の決定。people: セクションの中を見る。
    # 既定は start=0, end=一番短いクリップ長 (＝従来どおり全体をループ)。
    loop_start, loop_end = _parse_loop_window(people_cfg, loop_duration)

    log(f"spawned {spawned}/{requested} people from {path} "
        f"(loop window={loop_start:.2f}s..{loop_end:.2f}s)")
    return spawned, loop_start, loop_end
