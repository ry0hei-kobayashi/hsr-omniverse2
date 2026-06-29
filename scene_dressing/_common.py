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
from pxr import Gf, Sdf, UsdGeom, UsdShade


# 周囲背景画像 4 枚の組: (北 +Y, 南 -Y, 東 +X, 西 -X)
BackdropTextureSet = Tuple[str, str, str, str]


def log(message: str) -> None:
    print(f"[scene_dressing] {message}")


def ensure_xform(prim_path: str) -> None:
    """指定パスに Xform プリムが無ければ作る (冪等)。"""
    _stage = omni.usd.get_context().get_stage()
    if not _stage.GetPrimAtPath(prim_path).IsValid():
        create_prim(prim_path=prim_path, prim_type="Xform")


def set_translate(prim_path: str, x: float, y: float, z: float = 0.0) -> None:
    """prim の平行移動 (translate) を設定する (冪等)。

    既に translate op があれば値を上書きし、無ければ追加する。
    再実行で xformOp が重複しないようにするためのヘルパー。
    env_root をここで動かすと、その配下の床・背景幕・照明がまとめて移動する。
    """
    _stage = omni.usd.get_context().get_stage()
    xf = UsdGeom.Xformable(_stage.GetPrimAtPath(prim_path))
    op = next(
        (o for o in xf.GetOrderedXformOps()
         if o.GetOpType() == UsdGeom.XformOp.TypeTranslate),
        None,
    )
    if op is None:
        op = xf.AddTranslateOp()
    op.Set(Gf.Vec3d(x, y, z))


def bind_solid_color_material(
    prim_path: str,
    color: Tuple[float, float, float],
    roughness: float = 0.85,
) -> None:
    """単色 (テクスチャ無し) の UsdPreviewSurface を作って prim にバインドする。

    壁などを「塗り壁」っぽい一色で塗りたいときに使う。テクスチャ画像が要らない
    ぶん bind_uv_texture_material より手軽。UV (st) も不要なので Cube などの
    プリミティブにもそのまま使える。

    Args:
        prim_path: バインド先のプリム。
        color:     拡散色 (R, G, B)。各 0.0〜1.0。
        roughness: 表面のざらつき (0=鏡面〜1=完全マット)。塗り壁はマット寄り。
    """
    _stage = omni.usd.get_context().get_stage()
    mat_path = f"{prim_path}_mat"
    material = UsdShade.Material.Define(_stage, mat_path)

    shader = UsdShade.Shader.Define(_stage, mat_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*color)
    )
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    UsdShade.MaterialBindingAPI(_stage.GetPrimAtPath(prim_path)).Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
    )


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
