#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
dressing_presets: configs/dressing.yaml からプリセットを読み込むモジュール

リリース後の設定変更はこのファイルではなく configs/dressing.yaml で行うこと。
このモジュールは YAML を読み込んで Python 辞書として公開するだけ。

公開:
  - DRESSING_PRESETS: テクスチャプリセット辞書
  - LIGHTING_PRESETS: 照明プリセット辞書
  - DEFAULT_PRESET:   apply_lab_dressing() の preset 引数省略時の名前
  - DEFAULT_LIGHTING: apply_lab_dressing() の lighting 引数省略時の名前
  - list_presets():   利用可能なプリセット一覧 (デバッグ用)
  - reload():         YAML を再読み込み (テスト・対話用)
"""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

import yaml


# ============================================================
# 設定ファイルの場所
# ============================================================
# デフォルトは /app/configs/dressing.yaml (docker-compose で bind mount)。
# 環境変数 DRESSING_CONFIG で上書き可能。

DEFAULT_CONFIG_PATH: str = "/app/configs/dressing.yaml"
CONFIG_PATH: str = os.environ.get("DRESSING_CONFIG", DEFAULT_CONFIG_PATH)


# ============================================================
# 内部: YAML 読み込み + 正規化
# ============================================================

def _normalize_preset(preset: Dict[str, Any]) -> Dict[str, Any]:
    """YAML から読んだプリセット辞書を内部で扱いやすい形に整える。

    - backdrop_textures: list → tuple (scene_dressing 側が tuple 想定)
    - 文字列で来た数値 (YAML 1.1 で 6e5 等が string 扱いされた場合) を float に
    """
    normalized = dict(preset)

    # backdrop_textures は tuple に変換
    bts = normalized.get("backdrop_textures")
    if bts is not None and not isinstance(bts, tuple):
        normalized["backdrop_textures"] = tuple(bts)

    # 数値フィールドが文字列で来ていたら float 化
    float_fields = (
        "floor_tile", "floor_z", "room_size", "room_height",
        "dome_intensity", "ceiling_intensity", "ceiling_height",
        "ceiling_color_temp", "default_lights_intensity",
    )
    for k in float_fields:
        if k in normalized and normalized[k] is not None:
            normalized[k] = float(normalized[k])

    return normalized


def _load_config(path: str) -> Dict[str, Any]:
    """YAML 設定ファイルを読み込んで dict を返す。

    ファイルが見つからない / パースに失敗した場合は例外を投げる
    (黙ってデフォルトに fallback すると気づけないため)。
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"dressing config not found at {path}. "
            f"docker-compose で configs/ を bind mount しているか確認してください。"
        )
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    # 各プリセットを正規化
    dressing_presets_raw = data.get("dressing_presets", {})
    lighting_presets_raw = data.get("lighting_presets", {})

    return {
        "dressing_presets": {
            name: _normalize_preset(p) for name, p in dressing_presets_raw.items()
        },
        "lighting_presets": {
            name: _normalize_preset(p) for name, p in lighting_presets_raw.items()
        },
        "defaults": data.get("defaults", {}),
    }


# ============================================================
# 読み込み (モジュール import 時に 1 回)
# ============================================================

_config: Dict[str, Any] = _load_config(CONFIG_PATH)
print(f"[dressing_presets] loaded from {CONFIG_PATH} "
      f"(dressing={list(_config['dressing_presets'].keys())}, "
      f"lighting={list(_config['lighting_presets'].keys())})")

DRESSING_PRESETS: Dict[str, Dict[str, Any]] = _config["dressing_presets"]
LIGHTING_PRESETS: Dict[str, Dict[str, Any]] = _config["lighting_presets"]
DEFAULT_PRESET: str = _config["defaults"].get("preset", "lab")
DEFAULT_LIGHTING: str = _config["defaults"].get("lighting", "default")


# ============================================================
# 公開 API
# ============================================================

def list_presets() -> Dict[str, List[str]]:
    """利用可能なプリセット一覧を返す (デバッグ・ドキュメント用)。"""
    return {
        "dressing": list(DRESSING_PRESETS.keys()),
        "lighting": list(LIGHTING_PRESETS.keys()),
    }


def reload(path: Optional[str] = None) -> None:
    """YAML を再読み込みする (テスト・対話セッション用)。

    通常は import 時に 1 回読み込まれるだけだが、設定を編集してから
    Isaac Sim を再起動せずに反映したいときに使う。
    """
    global _config, DRESSING_PRESETS, LIGHTING_PRESETS
    global DEFAULT_PRESET, DEFAULT_LIGHTING
    target_path = path or CONFIG_PATH
    _config = _load_config(target_path)
    DRESSING_PRESETS = _config["dressing_presets"]
    LIGHTING_PRESETS = _config["lighting_presets"]
    DEFAULT_PRESET = _config["defaults"].get("preset", "lab")
    DEFAULT_LIGHTING = _config["defaults"].get("lighting", "default")
    print(f"[dressing_presets] reloaded from {target_path}")
