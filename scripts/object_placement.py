#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
object_placement: configs/placement.yaml に従って家具の上に物体を配置する。

generate_wrs_task のランダム配置を「置き換える」モジュール。
world ファイルから各家具 (unit_box) の天板の高さを計算し、YAML で指定された
物体をその天板の少し上にスポーンして、物理で自然に着地させる。

公開:
  - apply_placements(world_file, drop_func, config_path=None):
        設定ファイルを読み、家具ごとに物体を drop_func で配置する。
  - load_config(path):   YAML を辞書として読み込む (デバッグ用)
  - read_furniture(world_file): 家具名 -> 形状 を返す (デバッグ用)

drop_func は launch_isaacsim.py の drop_object と同じシグネチャを想定:
    drop_func(gazebo_name, name, x, y, z, yaw=0.0, roll=0.0, pitch=0.0)
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from typing import Any, Callable, Dict, Tuple

import yaml


# ============================================================
# 設定ファイルの場所
# ============================================================
# デフォルトは /app/configs/placement.yaml (docker-compose で bind mount)。
# 環境変数 PLACEMENT_CONFIG で上書き可能。

DEFAULT_CONFIG_PATH: str = "/app/configs/placement.yaml"
CONFIG_PATH: str = os.environ.get("PLACEMENT_CONFIG", DEFAULT_CONFIG_PATH)


def log(message: str) -> None:
    print(f"[placement] {message}")


# ============================================================
# YAML 読み込み
# ============================================================

def load_config(path: str) -> Dict[str, Any]:
    """placement.yaml を辞書として読み込む。

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


# ============================================================
# world ファイルから家具の形状を読む
# ============================================================

def read_furniture(world_file: str) -> Dict[str, Tuple[float, float, float, float, float]]:
    """world ファイルの <include> から家具を読み、

        家具名 -> (x, y, z, yaw, height)

    の辞書を返す。height は unit_box の scale の z 成分 (= 家具の高さ)。
    scale を持たない include (= 通常の model.usd 家具) はスキップする。
    """
    tree = ET.parse(world_file)
    root = tree.getroot()
    furniture: Dict[str, Tuple[float, float, float, float, float]] = {}
    for inc in root.findall("world/include"):
        name_tag = inc.find("name")
        pose_tag = inc.find("pose")
        scale_tag = inc.find("scale")
        if name_tag is None or pose_tag is None or scale_tag is None:
            continue
        name = name_tag.text
        pose = [float(n) for n in pose_tag.text.split()]
        x, y, z = pose[0], pose[1], pose[2]
        yaw = pose[5]  # pose は x y z roll pitch yaw
        scale = [float(n) for n in scale_tag.text.split()]
        height = scale[2]
        furniture[name] = (x, y, z, yaw, height)
    return furniture


# ============================================================
# 配置本体
# ============================================================

def _parse_item(item: Any) -> Dict[str, Any]:
    """YAML の 1 エントリを正規化する。

    文字列なら物体名だけ、辞書なら object/dx/dy/yaw/roll/pitch を読む。
    """
    if isinstance(item, str):
        return {"object": item, "dx": 0.0, "dy": 0.0,
                "yaw": 0.0, "roll": 0.0, "pitch": 0.0}
    return {
        "object": item["object"],
        "dx": float(item.get("dx", 0.0)),
        "dy": float(item.get("dy", 0.0)),
        "yaw": float(item.get("yaw", 0.0)),
        "roll": float(item.get("roll", 0.0)),
        "pitch": float(item.get("pitch", 0.0)),
    }


def apply_placements(
    world_file: str,
    drop_func: Callable[..., None],
    config_path: str | None = None,
) -> int:
    """placement.yaml に従って物体を配置する。配置した個数を返す。

    Args:
        world_file: 家具の位置・大きさが書かれた .world ファイルのパス
        drop_func:  物体をスポーンする関数 (launch_isaacsim.py の drop_object)
        config_path: 設定ファイル。省略時は CONFIG_PATH。
    """
    path = config_path or CONFIG_PATH
    cfg = load_config(path)
    placements = cfg.get("placements") or {}
    clearance = float(cfg.get("drop_clearance", 0.05))

    if not placements:
        log("WARNING: 'placements' が空です。配置する物体がありません。")
        return 0

    furniture = read_furniture(world_file)

    requested = 0   # 設定で要求された物体数
    placed = 0      # 実際に配置できた数
    failed = []     # 見つからなかった物体名
    for furn_name, items in placements.items():
        if furn_name not in furniture:
            log(f'WARNING: 家具 "{furn_name}" が world に見つかりません。スキップ。')
            continue
        if not items:
            continue
        fx, fy, fz, fyaw, fheight = furniture[furn_name]
        top_z = fz + fheight / 2.0  # 天板の高さ

        for idx, raw in enumerate(items):
            it = _parse_item(raw)
            obj_name = it["object"]
            dx, dy = it["dx"], it["dy"]

            # 天板中央からのオフセットを家具の向き (fyaw) で回して world 座標へ
            wx = fx + dx * math.cos(fyaw) - dy * math.sin(fyaw)
            wy = fy + dx * math.sin(fyaw) + dy * math.cos(fyaw)
            wz = top_z + clearance

            # prim パスが衝突しないよう一意な名前にする
            gazebo_name = f"{furn_name}__{obj_name}__{idx}"
            requested += 1
            # drop_func は成功で model.usd のパス、失敗 (モデル不在) で None を返す。
            result = drop_func(
                gazebo_name,
                obj_name,
                wx, wy, wz,
                yaw=fyaw + it["yaw"],
                roll=it["roll"],
                pitch=it["pitch"],
            )
            if result:
                placed += 1
            else:
                failed.append(f"{furn_name}/{obj_name}")

    log(f"placed {placed}/{requested} objects from {path}")
    if failed:
        log(f"WARNING: モデルが見つからず配置できなかった物体: {failed}")
    return placed
