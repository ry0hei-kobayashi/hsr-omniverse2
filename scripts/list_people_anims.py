#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
list_people_anims: アセットサーバにある「人のアニメ」ファイル一覧を出すだけの確認用。

なぜ作るか:
  placement.yaml の people.list[].motion 名 (waving など) は、最終的に
  /Isaac/People/Animations/<名前>.skelanim.usd というファイルを読みに行く。
  ところが "waving" だけ Tポーズ (＝アニメが当たっていない) になった。
  原因の第一候補は「そのファイル名が実在しない」こと。
  そこで実際に入っているファイル名を一覧して、正しい名前を確かめる。

特徴:
  - headless (画面なし) で起動するので、本番ループのような描画クラッシュは起きない。
  - シミュレーションは一切回さない。フォルダの中身を表示して終了するだけ。

実行 (dev モードでコンテナが起動している状態で):
  make ros2 dev list-anims
"""
from __future__ import annotations

# 画面を出さない (headless) で Isaac を最小起動する。
from isaacsim.simulation_app import SimulationApp

kit = SimulationApp({"headless": True})

import omni.client  # noqa: E402  (SimulationApp 起動後でないと import できない)
from isaacsim.storage.native import get_assets_root_path  # noqa: E402

# people_spawn.py と同じ場所を見る。
ANIM_DIR = "/Isaac/People/Animations"


def main() -> None:
    assets_root = get_assets_root_path()
    if not assets_root:
        print("[list-anims] ERROR: アセットサーバの場所が取得できませんでした。")
        return

    url = f"{assets_root}{ANIM_DIR}"
    print(f"[list-anims] 一覧する場所: {url}")

    # omni.client.list はフォルダの中身を (結果コード, エントリ一覧) で返す。
    result, entries = omni.client.list(url)
    names = sorted(e.relative_path for e in entries)

    if not names:
        print(f"[list-anims] ファイルが見つかりません (result={result})。")
        return

    print(f"[list-anims] 見つかったファイル数: {len(names)}")
    print("-" * 50)
    for name in names:
        # 手を振る系を見つけやすいよう、それっぽい名前に印を付ける。
        mark = ""
        low = name.lower()
        if "wav" in low or "wave" in low or "hello" in low or "hi" in low:
            mark = "   <-- 手を振る系かも"
        print(f"  {name}{mark}")
    print("-" * 50)
    print('[list-anims] placement.yaml の motion には拡張子 (.skelanim.usd) を'
          '除いた名前を書きます。')


main()
kit.close()
