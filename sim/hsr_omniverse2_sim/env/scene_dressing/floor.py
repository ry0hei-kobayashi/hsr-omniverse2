#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
scene_dressing.floor: 床テクスチャモジュール

X-Y 平面上に UV 付きメッシュを置き、UsdPreviewSurface + UsdUVTexture で
テクスチャを貼る。既存の床ビジュアル (Grid env など) を覆い隠す前提で
z をわずかに浮かせて重ねる。
"""
from __future__ import annotations

import os

import omni.usd
from pxr import Gf, Sdf, UsdGeom

from ._common import bind_uv_texture_material, log


def create_env_floor(
    prim_path: str,
    size: float,
    z: float,
    texture_path: str,
    tile: float,
) -> bool:
    """X-Y 平面に UV 付き 4 頂点メッシュを作りテクスチャを貼る。

    Args:
        prim_path:    生成するメッシュの USD プリムパス。
        size:         一辺の長さ (m)。
        z:            床の Z 座標 (m)。既存床との z-fight 回避用にわずかに浮かす。
        texture_path: 床テクスチャ画像のファイルパス。
        tile:         UV 繰り返し回数 (大きいほど細かいタイル)。

    Returns:
        bool: 作成に成功したら True、テクスチャが見つからなければ False。
    """
    _stage = omni.usd.get_context().get_stage()
    if not os.path.isfile(texture_path):
        log(f"床テクスチャが見つかりません: {texture_path}")
        return False

    mesh = UsdGeom.Mesh.Define(_stage, prim_path)
    half = size / 2.0
    mesh.CreatePointsAttr([
        Gf.Vec3f(-half, -half, 0.0),
        Gf.Vec3f( half, -half, 0.0),
        Gf.Vec3f( half,  half, 0.0),
        Gf.Vec3f(-half,  half, 0.0),
    ])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0.0, 0.0, 1.0)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex,
    )
    st.Set([
        Gf.Vec2f(0.0,  0.0),
        Gf.Vec2f(tile, 0.0),
        Gf.Vec2f(tile, tile),
        Gf.Vec2f(0.0,  tile),
    ])

    xf = UsdGeom.Xformable(mesh.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z))

    # 床はタイリングするので repeat
    bind_uv_texture_material(prim_path, texture_path, wrap="repeat")

    log(f"床作成: {prim_path} {size:.1f}×{size:.1f}m z={z} tile={tile}")
    return True
