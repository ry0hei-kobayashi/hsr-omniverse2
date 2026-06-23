#!/usr/bin/env python3
"""スキャン生成オブジェクトの USD を YCB 相当の物理構造へ自動変換する。

iPhone の Object Capture / Scaniverse などで作った model.usd は、メッシュが 1 個
あるだけで物理(剛体・当たり判定)を持たない。本スクリプトはそれを、YCB の model.usd
と同じ構造:

    /<root>/body                <PhysicsRigidBodyAPI, PhysicsMassAPI>   (+ physics:mass)
    /<root>/body/visuals        見た目用メッシュ (マテリアル維持)
    /<root>/body/collisions     当たり判定用メッシュ (CollisionAPI + MeshCollisionAPI, convexHull)

へ書き換える。元のメッシュ 1 個を body の下の visuals / collisions へ複製する。

使い方:
    # 単体
    python3 scripts/convert_object_to_ycb_physics.py usd/rc26_practice_day_1/drink/milk/model.usd
    # 複数 / ディレクトリ (配下の model.usd を再帰的に探す)
    python3 scripts/convert_object_to_ycb_physics.py usd/rc26_practice_day_1
    # 質量や近似を指定
    python3 scripts/convert_object_to_ycb_physics.py --mass 0.3 --approximation convexHull <path...>
    # 変換せず対象だけ表示
    python3 scripts/convert_object_to_ycb_physics.py --dry-run usd/rc26_practice_day_1

既に body/collisions を持つ USD (YCB や変換済み) はスキップする(冪等)。
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional

from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

DEFAULT_MASS_KG = 0.2
DEFAULT_APPROX = "convexHull"


def _find_source_mesh(stage: Usd.Stage, body_path: Sdf.Path) -> Optional[Usd.Prim]:
    """物理を付ける元メッシュを探す。body 配下のものは除外する。"""
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Mesh":
            continue
        if prim.GetPath().HasPrefix(body_path):
            continue
        return prim
    return None


def _already_converted(stage: Usd.Stage) -> bool:
    """既に剛体 + 当たり判定を持っているか(YCB / 変換済み)。"""
    has_rigid = any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse())
    has_col = any(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse())
    return has_rigid and has_col


def convert(usd_path: str, mass_kg: float = DEFAULT_MASS_KG,
            approximation: str = DEFAULT_APPROX) -> str:
    """1 つの model.usd を YCB 相当構造へ変換する。結果メッセージを返す。"""
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        return f"OPEN FAILED: {usd_path}"

    if _already_converted(stage):
        return f"skip (already has rigid+collision): {usd_path}"

    root = stage.GetDefaultPrim()
    if not root or not root.IsValid():
        return f"skip (no defaultPrim): {usd_path}"
    root_path = root.GetPath()
    body_path = root_path.AppendChild("body")

    src_mesh = _find_source_mesh(stage, body_path)
    if src_mesh is None:
        return f"skip (no mesh found): {usd_path}"
    src_path = src_mesh.GetPath()

    # 1) body (剛体 + 質量)
    body = UsdGeom.Xform.Define(stage, body_path)
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(body.GetPrim())
    mass_api.CreateMassAttr(float(mass_kg))

    # 2) 元メッシュを visuals / collisions へ複製 (points/uv/material 等まるごと)
    layer = stage.GetRootLayer()
    vis_path = body_path.AppendChild("visuals")
    col_path = body_path.AppendChild("collisions")
    if not Sdf.CopySpec(layer, src_path, layer, vis_path):
        return f"FAILED CopySpec visuals: {usd_path}"
    if not Sdf.CopySpec(layer, src_path, layer, col_path):
        return f"FAILED CopySpec collisions: {usd_path}"

    # 3) collisions: マテリアル束縛を外し、当たり判定 API を付ける
    col = stage.GetPrimAtPath(col_path)
    if col.HasRelationship("material:binding"):
        col.RemoveProperty("material:binding")
    try:
        col.RemoveAPI(UsdShade.MaterialBindingAPI)
    except Exception:
        col.RemoveAppliedSchema("MaterialBindingAPI")
    UsdPhysics.CollisionAPI.Apply(col)
    mesh_col = UsdPhysics.MeshCollisionAPI.Apply(col)
    mesh_col.CreateApproximationAttr(approximation)

    # 4) 元メッシュは削除 (空の Geometry scope は残す = YCB/手編集版と同じ)
    stage.RemovePrim(src_path)

    layer.Save()
    return f"converted (mass={mass_kg}, {approximation}): {usd_path}"


def _collect_usd_paths(paths: List[str]) -> List[str]:
    """ファイル/ディレクトリ混在の引数から model.usd のリストを作る。"""
    out: List[str] = []
    for p in paths:
        if os.path.isdir(p):
            for dirpath, _dirs, files in os.walk(p):
                for f in files:
                    if f == "model.usd":
                        out.append(os.path.join(dirpath, f))
        elif os.path.isfile(p):
            out.append(p)
        else:
            print(f"WARNING: not found: {p}", file=sys.stderr)
    return sorted(set(out))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="model.usd か、それを含むディレクトリ")
    ap.add_argument("--mass", type=float, default=DEFAULT_MASS_KG,
                    help=f"既定質量 kg (default: {DEFAULT_MASS_KG})")
    ap.add_argument("--approximation", default=DEFAULT_APPROX,
                    help=f"当たり判定の近似 (default: {DEFAULT_APPROX})")
    ap.add_argument("--dry-run", action="store_true",
                    help="変換せず対象一覧だけ表示")
    args = ap.parse_args()

    usd_paths = _collect_usd_paths(args.paths)
    if not usd_paths:
        print("対象の model.usd が見つかりません", file=sys.stderr)
        return 1

    print(f"対象 {len(usd_paths)} 件:")
    if args.dry_run:
        for p in usd_paths:
            print(f"  {p}")
        return 0

    for p in usd_paths:
        print("  " + convert(p, mass_kg=args.mass, approximation=args.approximation))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
