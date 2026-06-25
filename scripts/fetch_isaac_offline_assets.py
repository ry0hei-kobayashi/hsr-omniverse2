#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
fetch_isaac_offline_assets: 会場(WiFiなし)で完全オフライン動作させるため、
人(キャラ+アニメ)など「アセットサーバ依存」の USD 一式を、依存ファイルごと
ローカル(usd/isaac_offline/)にミラー(コピー)してくる一回限りの取得ツール。

なぜ必要か:
  これまで people_spawn.py は Biped_Setup.usd やアニメ(.skelanim.usd)を NVIDIA の
  アセットサーバ(ネット)から実行時に取得していた。会場ではネットが使えないので、
  あらかじめ必要な USD を「サーバと同じフォルダ構成のまま」ローカルへコピーしておき、
  実行時はそこから読む(people_spawn.py / furniture_spawn.py が usd/isaac_offline/ を優先)。

  ポイント: USD は他のファイル(メッシュ・テクスチャ・アニメ)を参照しているので、
  単体をコピーするだけでは足りない。本ツールは参照を再帰的にたどって「依存ファイル
  一式(=閉包)」を漏れなく集める。手を振る等のアニメは数値データの .skelanim.usd で、
  一度ローカルに置けば再生中にネットは要らない(=オフラインでモーションも動く)。

使い方 (オンライン環境で一度だけ実行する):
  推奨は Makefile 経由:
      make ros2 offline-assets        # 既定(人アセット)を usd/isaac_offline/ に取得
  直接実行する場合は Isaac コンテナ内(pxr が使える環境)で:
      /isaac-sim/kit/python/bin/python3 scripts/fetch_isaac_offline_assets.py \
          --out /path/to/repo/usd/isaac_offline

  別アセット(例: サーバ上の家具)も足したいときは --path で追加できる:
      ... fetch_isaac_offline_assets.py --out <out> \
          --path /NVIDIA/Assets/ArchVis/Residential/Furniture/.../Table.usd

注意:
  - ネットに繋がる状態で実行すること(会場では実行しない。会場前に済ませておく)。
  - 取得後は usd/isaac_offline/ をリポジトリにコミットすれば、以後はネット不要。
"""
from __future__ import annotations

import argparse
import os
import posixpath
import sys
import urllib.request


# ============================================================
# pxr(USD ライブラリ) の自動セットアップ
# ============================================================
# Isaac 4.5 の標準 python.sh では pxr が import できないため、拡張キャッシュにある
# omni.usd.libs を見つけて PYTHONPATH / LD_LIBRARY_PATH に足し、自分を再起動する。
# (Makefile から環境変数を渡して実行する場合は、この処理は素通りする。)
def _bootstrap_pxr() -> None:
    try:
        import pxr  # noqa: F401
        return
    except ImportError:
        pass
    if os.environ.get("_PXR_BOOTSTRAPPED") == "1":
        raise SystemExit(
            "pxr(USD) を読み込めませんでした。Isaac コンテナ内で実行してください。")
    import glob
    cands = sorted(glob.glob("/isaac-sim/extscache/omni.usd.libs-*/"))
    if not cands:
        raise SystemExit(
            "omni.usd.libs が見つかりません。Isaac Sim コンテナ内で実行してください。")
    libs = cands[-1].rstrip("/")
    os.environ["PYTHONPATH"] = libs + os.pathsep + os.environ.get("PYTHONPATH", "")
    os.environ["LD_LIBRARY_PATH"] = (
        os.path.join(libs, "bin") + os.pathsep + os.environ.get("LD_LIBRARY_PATH", ""))
    os.environ["_PXR_BOOTSTRAPPED"] = "1"
    os.execv(sys.executable, [sys.executable] + sys.argv)


_bootstrap_pxr()
from pxr import Sdf  # noqa: E402  (bootstrap 後でないと import できない)


# ============================================================
# 設定
# ============================================================
# アセットサーバの URL ルート (Isaac 4.5)。別バージョンは --base で上書き可。
DEFAULT_BASE = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5")

# 既定の取得対象(=起点)。people_spawn.py が読むキャラ本体と、MOTION_FILES のアニメ。
# ※ people_spawn.py の CHARACTER_USD / MOTION_FILES と内容を一致させること。
DEFAULT_SEEDS = [
    "/Isaac/People/Characters/Biped_Setup.usd",
    "/Isaac/People/Animations/stand_idle_loop.skelanim.usd",
    "/Isaac/People/Animations/stand_idle_wave_loop.skelanim.usd",
    "/Isaac/People/Animations/LookAround.skelanim.usd",
    "/Isaac/People/Animations/Sit.skelanim.usd",
    "/Isaac/People/Animations/stand_walk_loop.skelanim.usd",
    "/Isaac/People/Animations/push_button.skelanim.usd",
    "/Isaac/People/Animations/type_keyboard.skelanim.usd",
]

USD_EXT = (".usd", ".usdc", ".usda", ".usdz")


def log(msg: str) -> None:
    print(f"[fetch-offline] {msg}")


# ============================================================
# 1 ファイルのダウンロード
# ============================================================
def download(base: str, out_root: str, server_path: str) -> tuple:
    """server_path を base からダウンロードして out_root 配下の同じ構成に保存する。"""
    url = base.rstrip("/") + server_path
    local = os.path.join(out_root, server_path.lstrip("/"))
    os.makedirs(os.path.dirname(local), exist_ok=True)
    if os.path.exists(local):
        return local, True
    try:
        urllib.request.urlretrieve(url, local)
        return local, True
    except Exception as ex:  # noqa: BLE001
        log(f"  !! ダウンロード失敗 {url}: {ex}")
        return local, False


# ============================================================
# USD レイヤの依存(参照・テクスチャ等)を取り出す
# ============================================================
def _asset_attr_paths(layer: "Sdf.Layer") -> list:
    """レイヤ内の asset 型アトリビュート(テクスチャ等)のパスを集める。"""
    out: list = []

    def walk(spec) -> None:
        for attr in getattr(spec, "attributes", []):
            if "asset" not in str(attr.typeName):
                continue
            d = attr.default
            if d is None:
                continue
            if isinstance(d, Sdf.AssetPath):
                if d.path:
                    out.append(d.path)
            else:
                try:  # asset[] (UDIM 等)
                    for a in d:
                        if isinstance(a, Sdf.AssetPath) and a.path:
                            out.append(a.path)
                except TypeError:
                    pass
        for ch in getattr(spec, "nameChildren", []):
            walk(ch)

    walk(layer.pseudoRoot)
    return out


def _resolve_dep(base: str, server_dir: str, dep: str):
    """参照 dep を「サーバ絶対パス(/Isaac/...)」に正規化する。外部URLは None。"""
    if dep.startswith(base):
        dep = dep[len(base):]
    if dep.startswith(("omniverse://", "http://", "https://")):
        return None
    if dep.startswith("/"):
        return posixpath.normpath(dep)
    return posixpath.normpath(posixpath.join(server_dir, dep))


# ============================================================
# 再帰的に閉包を集める
# ============================================================
def crawl(base: str, out_root: str, seeds: list) -> int:
    visited: set = set()
    failed: list = []
    queue = list(seeds)
    n_usd = n_other = 0
    while queue:
        sp = queue.pop()
        if sp in visited:
            continue
        visited.add(sp)
        local, ok = download(base, out_root, sp)
        if not ok:
            failed.append(sp)
            continue
        if os.path.splitext(sp)[1].lower() in USD_EXT:
            n_usd += 1
            lyr = Sdf.Layer.FindOrOpen(local)
            if lyr is None:
                log(f"  !! USD を開けません: {local}")
                failed.append(sp)
                continue
            deps = list(lyr.GetCompositionAssetDependencies()) + _asset_attr_paths(lyr)
            sdir = posixpath.dirname(sp)
            for d in deps:
                child = _resolve_dep(base, sdir, d)
                if child and child not in visited:
                    queue.append(child)
        else:
            n_other += 1

    log(f"取得完了: USD {n_usd} / その他(テクスチャ等) {n_other} / 合計 "
        f"{len(visited) - len(failed)} ファイル -> {out_root}")
    # OmniPBR.mdl のような Isaac 本体内蔵のコア材質は 404 になるが、オフラインでも
    # Isaac 側で解決されるため無視してよい(同梱不要)。
    benign = [f for f in failed if os.path.basename(f).lower() in ("omnipbr.mdl",)]
    real = [f for f in failed if f not in benign]
    if benign:
        log(f"(無視OK) Isaac 内蔵で解決される参照: {', '.join(benign)}")
    if real:
        log(f"WARNING: 取得できなかった依存が {len(real)} 件あります:")
        for f in real:
            log(f"   - {f}")
        return 1
    log("すべての依存をローカルに取得しました(オフライン閉包OK)。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True,
                    help="出力先(例: <repo>/usd/isaac_offline)。サーバと同じ構成で保存する。")
    ap.add_argument("--base", default=DEFAULT_BASE,
                    help=f"アセットサーバ URL ルート(既定: {DEFAULT_BASE})")
    ap.add_argument("--path", action="append", default=[],
                    help="既定の取得対象に追加するサーバ絶対パス(複数指定可)。")
    ap.add_argument("--only", action="store_true",
                    help="既定(人アセット)を取らず、--path で指定したものだけ取得する。")
    args = ap.parse_args()

    seeds = list(args.path) if args.only else (DEFAULT_SEEDS + list(args.path))
    if not seeds:
        log("取得対象がありません(--path を指定するか --only を外してください)。")
        return 2
    os.makedirs(args.out, exist_ok=True)
    log(f"取得開始: {len(seeds)} 個の起点 / base={args.base}")
    return crawl(args.base, args.out, seeds)


if __name__ == "__main__":
    raise SystemExit(main())
