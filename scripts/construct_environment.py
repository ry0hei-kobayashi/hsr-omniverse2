#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Optional, Tuple

import omni.kit.commands
import omni.usd
from omni.isaac.core.utils import stage, viewports
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from pxr import Gf, Sdf, Usd, UsdShade

import scene_dressing
from dressing_presets import (
    DEFAULT_LIGHTING,
    DEFAULT_PRESET,
    DRESSING_PRESETS,
    LIGHTING_PRESETS,
    list_presets,
)


OBJECT_COLLECTION_PATH = "/World/YcbObjects"
ENV_ROOT = "/World/Env"

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
MODEL_ROOT = os.path.join(REPO_ROOT, "usd", "wrs_models")


def log(message: str) -> None:
    print(f"[construct] {message}")


def _prim_exists(prim_path: str) -> bool:
    stage_handle = omni.usd.get_context().get_stage()
    prim = stage_handle.GetPrimAtPath(prim_path)
    return prim.IsValid()


def ensure_xform(prim_path: str) -> None:
    if not _prim_exists(prim_path):
        create_prim(prim_path, "Xform")


def setup_camera() -> None:
    viewports.set_camera_view(
        eye=(3.0, 1.5, 3.0),
        target=(0.0, 0.0, 0.8),
    )


def setup_lights() -> None:
    if not _prim_exists("/World/LightKey"):
        create_prim(
            "/World/LightKey",
            "SphereLight",
            translation=(2.0, 0.0, 5.0),
            attributes={
                "inputs:radius": 0.01,
                "inputs:intensity": 4.5e4,
            },
        )

    if not _prim_exists("/World/FillLight"):
        create_prim(
            "/World/FillLight",
            "SphereLight",
            translation=(-2.0, -1.5, 5.0),
            attributes={
                "inputs:radius": 0.01,
                "inputs:intensity": 3.0e4,
            },
        )


def setup_object_collection(object_collection_path: str = OBJECT_COLLECTION_PATH) -> None:
    ensure_xform(object_collection_path)


def get_model_usd_path(model_name: str) -> str:
    candidates = [
        os.path.join(MODEL_ROOT, model_name, "model.usd"),
        os.path.join(MODEL_ROOT, model_name, "model.model.usd"),
        os.path.join(MODEL_ROOT, model_name, f"{model_name}.usd"),
    ]

    for path in candidates:
        if os.path.exists(path):
            return path

    raise FileNotFoundError(
        f"USD not found for model '{model_name}'. Checked:\n" +
        "\n".join(candidates)
    )


def add_reference(
    usd_path: str,
    prim_path: str,
    translation: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotation_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> str:
    if not os.path.exists(usd_path):
        raise FileNotFoundError(f"USD not found: {usd_path}")

    if _prim_exists(prim_path):
        omni.kit.commands.execute("DeletePrims", paths=[prim_path])

    create_prim(
        prim_path,
        "Xform",
        translation=translation,
        orientation=euler_angles_to_quat(rotation_rpy),
    )
    stage.add_reference_to_stage(usd_path, Sdf.Path(prim_path))
    return prim_path


def add_model(
    model_name: str,
    prim_name: Optional[str] = None,
    pos: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> str:
    usd_path = get_model_usd_path(model_name)
    final_prim_name = prim_name or model_name
    prim_path = f"{ENV_ROOT}/{final_prim_name}"
    return add_reference(
        usd_path=usd_path,
        prim_path=prim_path,
        translation=pos,
        rotation_rpy=rpy,
    )


# wrs2020.world.xacro 相当の配置
WRS_LAYOUT: List[Dict[str, object]] = [
    {"name": "wrc_frame",             "pos": (0.0,   0.0, 0.0), "rpy": (0.0, 0.0,  0.0)},
    {"name": "wrc_bookshelf",         "pos": (2.7,  -1.0, 0.0), "rpy": (0.0, 0.0, -1.57)},
    {"name": "wrc_bin_black",         "pos": (-2.7, -1.7, 0.0), "rpy": (0.0, 0.0,  0.0)},
    {"name": "wrc_bin_green",         "pos": (-2.7, -1.2, 0.0), "rpy": (0.0, 0.0,  0.0)},
    {"name": "wrc_stair_like_drawer", "pos": (-2.7,  1.0, 0.0), "rpy": (0.0, 0.0,  0.0)},
    {"name": "trofast",               "pos": (-2.7,  0.67, 0.1), "rpy": (0.0, 0.0, 0.0), "prim_name": "trofast_1"},
    {"name": "trofast",               "pos": (-2.7,  1.0,  0.1), "rpy": (0.0, 0.0, 0.0), "prim_name": "trofast_2"},
    {"name": "trofast",               "pos": (-2.7,  1.0,  0.36),"rpy": (0.0, 0.0, 0.0), "prim_name": "trofast_3"},
    {"name": "wrc_tall_table",        "pos": (-0.3,  1.2, 0.0), "rpy": (0.0, 0.0,  0.0)},
    {"name": "wrc_long_table",        "pos": (-0.3,  0.2, 0.0), "rpy": (0.0, 0.0,  1.57)},
    {"name": "wrc_long_table",        "pos": (-2.7, -0.3, 0.0), "rpy": (0.0, 0.0,  1.57), "prim_name": "wrc_long_table_0"},
    {"name": "wrc_tray",              "pos": (-2.7, -0.75, 0.4), "rpy": (0.0, 0.0, 1.57), "prim_name": "wrc_tray_1"},
    {"name": "wrc_tray",              "pos": (-2.7, -0.45, 0.4), "rpy": (0.0, 0.0, 1.57), "prim_name": "wrc_tray_2"},
    {"name": "wrc_container_a",       "pos": (-2.7, -0.2, 0.4), "rpy": (0.0, 0.0,  0.0)},
    {"name": "wrc_container_b",       "pos": (-2.7,  0.1, 0.4), "rpy": (0.0, 0.0,  0.0)},
    {"name": "person_standing",       "pos": (0.7,   1.5, 0.0), "rpy": (0.0, 0.0,  0.0)},
    {"name": "person_standing",       "pos": (1.8,   1.5, 0.0), "rpy": (0.0, 0.0,  0.0), "prim_name": "person_standing_0"},
]


def setup_ground_plane() -> None:
    """
    wrs_models 側に ground plane USD がないので、
    Isaac 側で単純な床を作る。
    """
    ground_path = "/World/GroundPlane"
    if _prim_exists(ground_path):
        return

    create_prim(
        prim_path=ground_path,
        prim_type="Cube",
        translation=(0.0, 0.0, -0.005),
        scale=(10.0, 10.0, 0.01),
    )


def setup_wrs_world() -> None:
    ensure_xform(ENV_ROOT)
    setup_ground_plane()

    for entry in WRS_LAYOUT:
        model_name = str(entry["name"])
        prim_name = entry.get("prim_name")
        pos = entry["pos"]
        rpy = entry["rpy"]

        try:
            add_model(
                model_name=model_name,
                prim_name=str(prim_name) if prim_name is not None else None,
                pos=pos,   # type: ignore[arg-type]
                rpy=rpy,   # type: ignore[arg-type]
            )
            log(f"Added environment model: {model_name}")
        except Exception as ex:
            log(f"Failed to add environment model '{model_name}': {ex}")


def construct_environment(
    object_collection_path: str = OBJECT_COLLECTION_PATH,
) -> None:
    ensure_xform("/World")
    setup_camera()
    setup_lights()
    setup_wrs_world()
    setup_object_collection(object_collection_path)


def spawn_object(
    model_name: str,
    x_offset: float,
    y_offset: float,
    z_offset: float = 0.01,
    yaw: float = math.pi / 2.0,
    object_collection_path: str = OBJECT_COLLECTION_PATH,
) -> str:
    usd_path = get_model_usd_path(model_name)

    ensure_xform(object_collection_path)

    stage_path = f"{object_collection_path}/{model_name}"
    if _prim_exists(stage_path):
        omni.kit.commands.execute("DeletePrims", paths=[stage_path])

    create_prim(
        stage_path,
        "Xform",
        translation=(0.39 + x_offset, 0.11 + y_offset, z_offset),
        orientation=euler_angles_to_quat([0.0, 0.0, yaw]),
    )
    stage.add_reference_to_stage(usd_path, Sdf.Path(stage_path))
    return stage_path


def spawn_object_at_pose(
    model_name: str,
    translation: Tuple[float, float, float],
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    object_collection_path: str = OBJECT_COLLECTION_PATH,
    prim_name: Optional[str] = None,
) -> str:
    usd_path = get_model_usd_path(model_name)

    ensure_xform(object_collection_path)

    final_prim_name = prim_name or model_name
    stage_path = f"{object_collection_path}/{final_prim_name}"

    if _prim_exists(stage_path):
        omni.kit.commands.execute("DeletePrims", paths=[stage_path])

    create_prim(
        stage_path,
        "Xform",
        translation=translation,
        orientation=euler_angles_to_quat(rpy),
    )
    stage.add_reference_to_stage(usd_path, Sdf.Path(stage_path))
    return stage_path


def delete_object(stage_path: str) -> None:
    if _prim_exists(stage_path):
        omni.kit.commands.execute("DeletePrims", paths=[stage_path])


def clear_spawned_objects(
    object_collection_path: str = OBJECT_COLLECTION_PATH,
) -> None:
    stage_handle = omni.usd.get_context().get_stage()
    parent_prim = stage_handle.GetPrimAtPath(object_collection_path)
    if not parent_prim.IsValid():
        return

    child_paths = [str(child.GetPath()) for child in parent_prim.GetChildren()]
    if child_paths:
        omni.kit.commands.execute("DeletePrims", paths=child_paths)


def apply_lab_dressing(
    preset: str = DEFAULT_PRESET,
    lighting: str = DEFAULT_LIGHTING,
    **overrides: Any,
) -> None:
    """scene_dressing で床テクスチャ + 周囲背景画像 + 照明を適用する。

    Args:
        preset:    テクスチャプリセット名 (dressing_presets.DRESSING_PRESETS のキー)。
                   例: "lab", "floor_only", "backdrop_only" など。
        lighting:  照明プリセット名 (dressing_presets.LIGHTING_PRESETS のキー)。
                   例: "default", "bright", "dim", "warm", "off" など。
        **overrides: EnvBoxConfig の個別フィールドを直接上書き。
                     プリセットより優先される (= 一時的な微調整に便利)。

    使い方:
        apply_lab_dressing()                              # 全部デフォルト
        apply_lab_dressing(lighting="bright")             # ラボを明るく
        apply_lab_dressing(preset="office")               # 別テーマ
        apply_lab_dressing(ceiling_intensity=2.5e5)       # 微調整
        apply_lab_dressing(preset="lab",
                           floor_texture="/data/other.jpg")  # 床だけ差し替え

    重要: omni.timeline.get_timeline_interface().play() の "後" に呼ぶこと。
          PhysX セットアップ前に呼ぶとシーン状態が壊れることがある。
    """
    if preset not in DRESSING_PRESETS:
        raise KeyError(
            f"Unknown dressing preset '{preset}'. "
            f"Available: {list(DRESSING_PRESETS.keys())}"
        )
    if lighting not in LIGHTING_PRESETS:
        raise KeyError(
            f"Unknown lighting preset '{lighting}'. "
            f"Available: {list(LIGHTING_PRESETS.keys())}"
        )

    # auto-exposure 無効化 + 背景 USD の隠れたライト消灯
    # (これらがないとプリセット差が見えにくい / 全く見えない)
    _ensure_renderer_settings_for_lighting()
    _disable_background_lights()

    # プリセットを合成して上書きをかける (右側が強い)
    params: Dict[str, Any] = {}
    params.update(DRESSING_PRESETS[preset])
    params.update(LIGHTING_PRESETS[lighting])
    params.update(overrides)

    # 拡張キー (EnvBoxConfig 対象外) を取り出す
    default_lights_intensity = params.pop("default_lights_intensity", None)
    # wall_color: world の物理壁 (/wall_*) を塗る色。EnvBoxConfig には無いので pop。
    # 指定しているのは現状 restaurant プリセットのみ (他タスクは None で無変更)。
    wall_color = params.pop("wall_color", None)
    # map_box_color: 占有地図由来の壁/障害物 (/map_box_*) を塗る色。rc26_from_map.world
    # の灰色の壁を白くするために rc26_venue プリセットで使う。EnvBoxConfig 対象外なので pop。
    map_box_color = params.pop("map_box_color", None)

    log(f"apply_lab_dressing: preset='{preset}' lighting='{lighting}' "
        f"overrides={list(overrides.keys())}")
    config = scene_dressing.EnvBoxConfig(**params)
    scene_dressing.apply_env_box(config)

    # 既存ライト (/World/Light_1, /World/Light_2) の強度をプリセットに連動させる
    if default_lights_intensity is not None:
        _set_default_lights_intensity(float(default_lights_intensity))

    # world の物理壁 (/wall_*) を指定色 (塗り壁) に塗る
    if wall_color is not None:
        _recolor_walls(wall_color)
    # 占有地図由来の壁/障害物 (/map_box_*) を指定色に塗る (灰色→白 など)
    if map_box_color is not None:
        _recolor_walls(map_box_color, prefix="map_box_")


def _recolor_walls(color: Any, prefix: str = "wall_") -> None:
    """world のトップレベル prim (名前が prefix で始まるもの) を単色で塗る。

    launch_isaacsim.py が world の <include> から作る灰色の Cube に「塗り壁」風の
    単色マテリアルを貼って見た目を整える。物理 (collider) はそのままなので自己位置
    推定や当たり判定には影響しない。
      - prefix="wall_"    : 物理壁 (/wall_*)          … dressing.yaml の wall_color
      - prefix="map_box_" : 占有地図由来の壁/障害物 (/map_box_*) … map_box_color

    Args:
        color: 拡散色 [R, G, B] (各 0.0〜1.0)。
        prefix: 塗装対象とするトップレベル prim 名の接頭辞。
    """
    try:
        rgb = tuple(float(c) for c in color)
    except (TypeError, ValueError):
        log(f"色の形式が不正です (期待: [R, G, B]): {color!r}")
        return
    if len(rgb) != 3:
        log(f"色は 3 要素 [R, G, B] で指定してください: {color!r}")
        return

    _stage = omni.usd.get_context().get_stage()

    # 共有マテリアルを 1 つだけ作り、対象 prim 全部に貼る。map_box_* は数百個
    # あるため、prim ごとにマテリアルを作ると重い。接頭辞ごとに固定パスで1個。
    _safe = prefix.rstrip("_") or "wall"
    mat_path = f"/World/Looks/recolor_{_safe}"
    material = UsdShade.Material.Define(_stage, mat_path)
    shader = UsdShade.Shader.Define(_stage, mat_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*rgb))
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface")

    painted = 0
    for prim in _stage.GetPseudoRoot().GetChildren():
        name = prim.GetName()
        if not name.startswith(prefix):
            continue
        UsdShade.MaterialBindingAPI(prim).Bind(
            material, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        painted += 1
    log(f"塗装 (/{prefix}*): {painted} 個を共有マテリアル color={rgb} で塗りました")
    if painted == 0:
        log(f"塗る対象 (/{prefix}*) が見つかりませんでした")


def _set_default_lights_intensity(intensity: float) -> None:
    """launch_isaacsim.py が作る /World/Light_1, /World/Light_2 の強度を設定。

    scene_dressing のライティングプリセットが効いて見えるためには、既存ライトを
    プリセットに合わせて連動させる必要がある (固定 5e4 だとドミネートする)。
    """
    _stage = omni.usd.get_context().get_stage()
    for path in ("/World/Light_1", "/World/Light_2"):
        prim = _stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        attr = prim.GetAttribute("inputs:intensity")
        if not attr:
            continue
        attr.Set(intensity)
        log(f"既存ライト強度設定: {path} = {intensity}")


_BG_LIGHT_TYPES = {
    "DomeLight", "DistantLight", "RectLight", "DiskLight",
    "SphereLight", "CylinderLight", "GeometryLight",
}


def _disable_background_lights() -> None:
    """背景 USD (Grid env など) に焼き込まれた隠れたライトを消灯する。

    Isaac Sim の default_environment.usd は DomeLight (sky HDRI) や DistantLight
    を内蔵していて、これが scene_dressing 側のライティングプリセットを覆い隠して
    ドミネートする (intensity を変えても見た目が変わらない原因)。
    /background 配下のライト型プリムを全部 0 にしてプリセットを効かせる。
    """
    _stage = omni.usd.get_context().get_stage()
    bg_root = _stage.GetPrimAtPath("/background")
    if not bg_root.IsValid():
        return

    disabled = 0
    for prim in Usd.PrimRange(bg_root):
        if prim.GetTypeName() not in _BG_LIGHT_TYPES:
            continue
        attr = prim.GetAttribute("inputs:intensity")
        if not attr:
            continue
        attr.Set(0.0)
        log(f"背景ライト消灯: {prim.GetPath()} ({prim.GetTypeName()})")
        disabled += 1
    if disabled == 0:
        log("背景に消すべきライトは見つかりませんでした")


_renderer_settings_applied = False


def _ensure_renderer_settings_for_lighting() -> None:
    """ライティングプリセットの差を視覚的に出すためのレンダラー設定 (初回 1 回だけ実行)。

    Isaac Sim の RTX レンダラーには 2 系統の自動露出補正がある:
      1. histogram-based auto-exposure (/rtx/post/histogram/enabled)
      2. camera-based eye adaptation (カメラの ISO/shutter/f-stop に基づく動的補正)

    両方とも無効化 + カメラ露出を固定値にして、ライティング強度変化が
    そのままピクセル値に反映されるようにする。
    """
    global _renderer_settings_applied
    if _renderer_settings_applied:
        return
    try:
        import carb.settings
        s = carb.settings.get_settings()

        # (1) histogram-based auto-exposure 無効
        s.set("/rtx/post/histogram/enabled", False)

        # (2) tonemap は Reinhard (1) に固定
        s.set("/rtx/post/tonemap/op", 1)

        # (3) カメラ露出を固定 (ISO 100, シャッター 1/60, F4)
        # これで eye adaptation 系の自動補正もキャンセル
        s.set("/rtx/post/tonemap/cameraIsoSensitivity", 100.0)
        s.set("/rtx/post/tonemap/cameraShutterSpeed", 60.0)
        s.set("/rtx/post/tonemap/cameraFStop", 4.0)

        log("renderer: auto-exposure OFF, tonemap=Reinhard, "
            "camera fixed (ISO 100, 1/60 sec, F4)")
        _renderer_settings_applied = True
    except Exception as e:
        log(f"renderer settings setup failed: {e}")
