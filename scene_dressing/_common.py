#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
scene_dressing._common: パッケージ内部で共有するユーティリティ

外から使うことを想定していない private モジュール。
"""
from __future__ import annotations

from typing import Tuple

import omni.usd
from omni.isaac.core.utils.prims import create_prim
from pxr import Sdf, UsdShade


# 周囲背景画像 4 枚の組: (北 +Y, 南 -Y, 東 +X, 西 -X)
BackdropTextureSet = Tuple[str, str, str, str]


def log(message: str) -> None:
    print(f"[scene_dressing] {message}")


def ensure_xform(prim_path: str) -> None:
    """指定パスに Xform プリムが無ければ作る (冪等)。"""
    _stage = omni.usd.get_context().get_stage()
    if not _stage.GetPrimAtPath(prim_path).IsValid():
        create_prim(prim_path=prim_path, prim_type="Xform")


def bind_uv_texture_material(
    prim_path: str,
    texture_path: str,
    wrap: str,
) -> None:
    """UsdPreviewSurface + UsdUVTexture マテリアルを作って prim にバインドする。

    Args:
        prim_path: バインド先のメッシュプリム。
        texture_path: テクスチャ画像のファイルパス。
        wrap: "clamp" (引き伸ばし) or "repeat" (タイリング)。
    """
    _stage = omni.usd.get_context().get_stage()
    mat_path = f"{prim_path}_mat"
    material = UsdShade.Material.Define(_stage, mat_path)

    shader = UsdShade.Shader.Define(_stage, mat_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    uv_reader = UsdShade.Shader.Define(_stage, mat_path + "/UVReader")
    uv_reader.CreateIdAttr("UsdPrimvarReader_float2")
    uv_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    uv_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    tex = UsdShade.Shader.Define(_stage, mat_path + "/Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture_path))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        uv_reader.ConnectableAPI(), "result",
    )
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set(wrap)
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set(wrap)
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb",
    )
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    UsdShade.MaterialBindingAPI(_stage.GetPrimAtPath(prim_path)).Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
    )
