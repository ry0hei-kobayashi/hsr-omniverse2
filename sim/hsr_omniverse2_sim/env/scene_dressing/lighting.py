#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
scene_dressing.lighting: 照明モジュール

壁で囲まれた部屋を内部から明るくするための照明をセットする。

- DomeLight × 1: シーン全体のアンビエント (天井が無いので上から差し込む)
- SphereLight × N: 天井付近に格子状に配置する直接光源
"""
from __future__ import annotations

import omni.usd
from pxr import Gf, Sdf, UsdGeom

from ._common import ensure_xform, log


def setup_env_lighting(
    env_root: str,
    room_size: float,
    dome_intensity: float = 3000.0,
    ceiling_intensity: float = 1.5e5,
    ceiling_height: float = 3.7,
    ceiling_count: int = 4,
    ceiling_color_temp: float = 6500.0,
) -> None:
    """壁で囲まれた部屋を内部から明るくするための照明をセットする。

    Args:
        env_root:            配下に Lights/ を作る親 Xform のパス。
        room_size:           部屋サイズ。天井灯の配置幅 (=room_size/4) に使う。
        dome_intensity:      DomeLight 強度。
        ceiling_intensity:   1 灯あたりの SphereLight 強度。
        ceiling_height:      天井灯の Z 高さ (m)。壁高さより少し下が推奨。
        ceiling_count:       天井灯の数。<=4 は四隅、>4 は四隅+中央。
        ceiling_color_temp:  天井灯の色温度 (K)。6500K = 昼白色。
    """
    _stage = omni.usd.get_context().get_stage()
    lights_root = f"{env_root}/Lights"
    ensure_xform(lights_root)

    # DomeLight
    dome_path = f"{lights_root}/Dome"
    dome = _stage.DefinePrim(dome_path, "DomeLight")
    dome.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(dome_intensity)
    log(f"DomeLight: {dome_path} intensity={dome_intensity}")

    if ceiling_count <= 0:
        return

    offset = room_size / 4.0
    base_positions = [
        ( offset,  offset, ceiling_height),
        (-offset,  offset, ceiling_height),
        ( offset, -offset, ceiling_height),
        (-offset, -offset, ceiling_height),
    ]
    positions = list(base_positions[:ceiling_count])
    if ceiling_count > 4:
        positions.append((0.0, 0.0, ceiling_height))

    for i, pos in enumerate(positions):
        path = f"{lights_root}/Ceiling_{i}"
        lp = _stage.DefinePrim(path, "SphereLight")
        UsdGeom.Xformable(lp).AddTranslateOp().Set(Gf.Vec3d(*pos))
        lp.CreateAttribute("inputs:radius", Sdf.ValueTypeNames.Float).Set(0.15)
        lp.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(ceiling_intensity)
        lp.CreateAttribute("inputs:enableColorTemperature", Sdf.ValueTypeNames.Bool).Set(True)
        lp.CreateAttribute("inputs:colorTemperature", Sdf.ValueTypeNames.Float).Set(ceiling_color_temp)
        log(f"天井ライト: {path} pos={pos} intensity={ceiling_intensity}")
