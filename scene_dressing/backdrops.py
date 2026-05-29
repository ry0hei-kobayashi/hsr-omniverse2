#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
scene_dressing.backdrops: 周囲背景画像モジュール

シーンの周囲を 4 枚の画像 (北・南・東・西) で取り囲み、ドメインランダム化
や見栄え用途に使う。物理的な「壁」ではなく、撮影スタジオの背景幕のような
情景画像として機能する。

各背景幕は UV 付き 4 頂点メッシュ + UsdPreviewSurface でテクスチャを貼る。
"""
from __future__ import annotations

import os
from typing import Tuple

import omni.usd
from pxr import Gf, Sdf, UsdGeom

from ._common import BackdropTextureSet, bind_uv_texture_material, ensure_xform, log


def create_env_backdrop(
    prim_path: str,
    width: float,
    height: float,
    position: Tuple[float, float, float],
    rotation_euler_deg: Tuple[float, float, float],
    texture_path: str,
) -> bool:
    """UV 付き 4 頂点の長方形メッシュを 1 枚作りテクスチャを貼る。

    ローカルで X-Z 平面に長方形を作り (法線 +Y)、translate + rotateXYZ で配置する。

    Returns:
        bool: 作成に成功したら True、テクスチャが見つからなければ False。
    """
    _stage = omni.usd.get_context().get_stage()
    if not os.path.isfile(texture_path):
        log(f"背景テクスチャが見つかりません: {texture_path}")
        return False

    mesh = UsdGeom.Mesh.Define(_stage, prim_path)
    half_w = width / 2.0
    half_h = height / 2.0
    mesh.CreatePointsAttr([
        Gf.Vec3f(-half_w, 0.0, -half_h),
        Gf.Vec3f( half_w, 0.0, -half_h),
        Gf.Vec3f( half_w, 0.0,  half_h),
        Gf.Vec3f(-half_w, 0.0,  half_h),
    ])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0.0, 1.0, 0.0)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex,
    )
    st.Set([
        Gf.Vec2f(0.0, 0.0),
        Gf.Vec2f(1.0, 0.0),
        Gf.Vec2f(1.0, 1.0),
        Gf.Vec2f(0.0, 1.0),
    ])

    xf = UsdGeom.Xformable(mesh.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*position))
    xf.AddRotateXYZOp().Set(Gf.Vec3d(*rotation_euler_deg))

    # 背景幕は 1 枚を引き伸ばし
    bind_uv_texture_material(prim_path, texture_path, wrap="clamp")

    log(
        f"背景幕作成: {prim_path} size=({width:.1f}×{height:.1f}m) "
        f"pos={position} rot={rotation_euler_deg}"
    )
    return True


def setup_env_backdrops(
    env_root: str,
    room_size: float,
    room_height: float,
    backdrop_textures: BackdropTextureSet,
) -> int:
    """シーン全体を 4 枚の背景画像で取り囲む。

    各背景の法線が内側を向くよう Z 軸で回転して配置:
        backdrop_textures[0]: 北 y=+half, 180° (法線 -Y)
        backdrop_textures[1]: 南 y=-half,   0° (法線 +Y)
        backdrop_textures[2]: 東 x=+half,  90° (法線 -X)
        backdrop_textures[3]: 西 x=-half, -90° (法線 +X)

    Returns:
        int: 作成に成功した背景幕の枚数 (0..4)。
    """
    if len(backdrop_textures) < 4:
        log(f"背景テクスチャは 4 枚必要です (受け取った数: {len(backdrop_textures)})")
        return 0

    half = room_size / 2.0
    h = room_height
    backdrops = (
        ("Backdrop_N", (0.0,  half, h / 2.0), (0.0, 0.0, 180.0), backdrop_textures[0]),
        ("Backdrop_S", (0.0, -half, h / 2.0), (0.0, 0.0,   0.0), backdrop_textures[1]),
        ("Backdrop_E", ( half, 0.0, h / 2.0), (0.0, 0.0,  90.0), backdrop_textures[2]),
        ("Backdrop_W", (-half, 0.0, h / 2.0), (0.0, 0.0, -90.0), backdrop_textures[3]),
    )

    ensure_xform(env_root)
    created = 0
    for suffix, pos, rot, tex in backdrops:
        if create_env_backdrop(
            prim_path=f"{env_root}/{suffix}",
            width=room_size,
            height=h,
            position=pos,
            rotation_euler_deg=rot,
            texture_path=tex,
        ):
            created += 1
    log(f"背景幕作成完了: {room_size}m × {room_size}m × {h}m (背景 {created}/4 枚)")
    return created
