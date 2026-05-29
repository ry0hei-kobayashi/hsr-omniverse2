#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
scene_dressing: 環境ボックス (床 + 周囲背景 + 照明) を作る再利用可能パッケージ

シーン全体を画像で囲まれた部屋に演出する。物理的な「壁」ではなく、撮影
スタジオの背景幕に近いイメージで、ドメインランダム化や見栄え用途に使う。

サブモジュール:
  - scene_dressing.floor     : 床テクスチャ
  - scene_dressing.backdrops : 4 枚の周囲背景画像
  - scene_dressing.lighting  : 照明 (DomeLight + 天井 SphereLight)

公開 API:
  - EnvBoxConfig:               設定を一括で渡す dataclass
  - apply_env_box(cfg):         設定通りに環境ボックスを構築するトップレベル
  - create_env_floor():         床 1 枚 (= floor.create_env_floor)
  - create_env_backdrop():      背景 1 枚 (= backdrops.create_env_backdrop)
  - setup_env_backdrops():      4 背景一括 (= backdrops.setup_env_backdrops)
  - setup_env_lighting():       照明セット (= lighting.setup_env_lighting)

使い方:
    import scene_dressing
    cfg = scene_dressing.EnvBoxConfig(
        floor_texture="/data/Tex/floor.jpg",
        backdrop_textures=("n.jpg", "s.jpg", "e.jpg", "w.jpg"),
    )
    scene_dressing.apply_env_box(cfg)

重要:
  omni.timeline.get_timeline_interface().play() の "後" に呼ぶこと。
  PhysX セットアップ前にマテリアル作成コマンドを実行するとシーン状態が壊れる。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ._common import BackdropTextureSet, ensure_xform, log
from .floor import create_env_floor
from .backdrops import create_env_backdrop, setup_env_backdrops
from .lighting import setup_env_lighting


__all__ = [
    "EnvBoxConfig",
    "BackdropTextureSet",
    "apply_env_box",
    "create_env_floor",
    "create_env_backdrop",
    "setup_env_backdrops",
    "setup_env_lighting",
]


# ============================================================
# 設定
# ============================================================

@dataclass
class EnvBoxConfig:
    """環境ボックスの全設定。

    床と背景と照明は独立にオン/オフできる:
      - floor_texture=None       → 床作成スキップ
      - backdrop_textures=None   → 背景作成スキップ
      - enable_lighting=False    → 照明追加スキップ
    """
    # 配置
    env_root: str = "/World/EnvBox"
    room_size: float = 15.0       # X-Y 平面の一辺 (m)
    room_height: float = 4.0      # 背景幕の高さ (m)

    # 床
    floor_texture: Optional[str] = None
    floor_tile: float = 4.0       # UV 繰り返し回数
    floor_z: float = 0.002        # 既存床との z-fight 回避オフセット

    # 周囲背景
    backdrop_textures: Optional[BackdropTextureSet] = None

    # 照明
    enable_lighting: bool = True
    dome_intensity: float = 3000.0
    ceiling_intensity: float = 1.5e5
    ceiling_height: float = 3.7
    ceiling_count: int = 4
    ceiling_color_temp: float = 6500.0


# ============================================================
# トップレベル
# ============================================================

def apply_env_box(config: EnvBoxConfig) -> None:
    """設定 (EnvBoxConfig) を受け取って環境ボックス一式 (床+背景+照明) を構築する。

    None 指定された項目はスキップされるので、「床だけ」「背景だけ」など
    部分的にも作れる。
    """
    ensure_xform(config.env_root)

    if config.floor_texture:
        create_env_floor(
            prim_path=f"{config.env_root}/Floor",
            size=config.room_size,
            z=config.floor_z,
            texture_path=config.floor_texture,
            tile=config.floor_tile,
        )
    else:
        log("floor_texture 未指定: 床作成スキップ")

    if config.backdrop_textures:
        setup_env_backdrops(
            env_root=config.env_root,
            room_size=config.room_size,
            room_height=config.room_height,
            backdrop_textures=config.backdrop_textures,
        )
    else:
        log("backdrop_textures 未指定: 背景作成スキップ")

    if config.enable_lighting:
        setup_env_lighting(
            env_root=config.env_root,
            room_size=config.room_size,
            dome_intensity=config.dome_intensity,
            ceiling_intensity=config.ceiling_intensity,
            ceiling_height=config.ceiling_height,
            ceiling_count=config.ceiling_count,
            ceiling_color_temp=config.ceiling_color_temp,
        )
