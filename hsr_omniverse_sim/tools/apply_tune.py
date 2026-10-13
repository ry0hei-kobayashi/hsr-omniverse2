#!/usr/bin/env python3
"""make tune の調整結果をタスクの placement.yaml に反映する (`make tune-apply TASK=<name>`)。

`make tune TASK=<name>` は結果を recordings/tune/arena_cameras.yaml に書き出すだけで、
hsr_omniverse_sim/config/tasks/<name>/placement.yaml は書き換えない (コンテナからは読み取り専用で
マウントされているため)。このスクリプトはホスト側で動き、調整済みの
`arena_cameras:` ブロックだけをそのタスクの placement.yaml に差し替える。

書き込み先は環境変数 TASK で決まる (Makefile が TASK=<name> を渡す):
    hsr_omniverse_sim/config/tasks/<TASK>/placement.yaml
TASK が無い・フォルダや placement.yaml が無いときはエラーで止まる
(共通の hsr_omniverse_sim/config/placement.yaml には書かない)。

ロボットの初期位置 (robot:) は対象外。手で placement.yaml に書く。

コメントや他のセクション (objects: など) はそのまま残す。yaml をいったん
オブジェクトにして書き戻すとコメントが全部消えてしまうので、テキストの
ブロック単位で置き換えている。

書き込む前に、結果が
  - yaml として読めるか
  - 調整した値が正しく入ったか
  - 他のセクションが消えていないか
を検証し、ひとつでも失敗したら書き込まずに終了する。
"""
from __future__ import annotations

import os
import shutil
import sys
import time

import yaml

from importlib.resources import files

# ホスト側で `python3 -m hsr_omniverse_sim.tools.apply_tune` (リポジトリ直下で実行 = make tune-apply) として動く。Isaac には依存しない。
CONFIG_DIR = str(files('hsr_omniverse_sim') / 'config')
TUNED = os.path.abspath(os.path.join("recordings", "tune", "arena_cameras.yaml"))  # カレント (リポジトリ直下) の recordings/
BLOCKS = ("arena_cameras",)   # robot: は手書きなので触らない


def resolve_target() -> str:
    """環境変数 TASK から書き込み先 hsr_omniverse_sim/config/tasks/<TASK>/placement.yaml を決める。

    TASK が無い、タスクフォルダが無い、placement.yaml が無い、のどれかなら
    SystemExit で分かりやすく止める。
    """
    task = os.environ.get("TASK", "").strip()
    if not task:
        raise SystemExit(
            "[apply-tune] 環境変数 TASK がありません。\n"
            "[apply-tune] どのタスクの placement.yaml に反映するか指定してください: "
            "make tune-apply TASK=<name>")
    task_dir = os.path.join(CONFIG_DIR, "tasks", task)
    if not os.path.isdir(task_dir):
        raise SystemExit(
            f"[apply-tune] TASK='{task}' のフォルダがありません: {task_dir}")
    target = os.path.join(task_dir, "placement.yaml")
    if not os.path.isfile(target):
        raise SystemExit(
            f"[apply-tune] {target} がありません。\n"
            "[apply-tune] arena_cameras: を含む placement.yaml をタスクフォルダに置いてから"
            "実行してください (hsr_omniverse_sim/config/tasks/README.md 参照)。")
    return target


def find_block(text: str, key: str):
    """先頭が `key:` の行から、その中身 (字下げ行) の終わりまでを返す。

    戻り値は (開始行, 終了行, ブロックの文字列)。見つからなければ None。
    ブロックの上にあるコメントや、下に続く別のセクションには触らない。
    """
    lines = text.splitlines(keepends=True)
    start = next((i for i, l in enumerate(lines)
                  if l.startswith(key + ":")), None)
    if start is None:
        return None

    end = start + 1
    while end < len(lines) and (lines[end].strip() == ""
                                or lines[end][0] in " \t"):
        end += 1
    # 末尾の空行はブロックの外 (セクションの区切り) として残す
    while end > start + 1 and lines[end - 1].strip() == "":
        end -= 1
    return start, end, "".join(lines[start:end])


def main() -> int:
    target = resolve_target()
    target_rel = os.path.relpath(target)
    if not os.path.exists(TUNED):
        print(f"[apply-tune] 調整結果がありません: {TUNED}")
        print("[apply-tune] 先に `make tune TASK=<name>` で位置を調整してください。")
        return 1

    # placement.yaml のほうが新しいなら、調整結果は前の座標系のものかもしれない。
    # そのまま貼ると、あとから行った変更 (原点の移動など) が巻き戻る。
    if os.path.getmtime(TUNED) < os.path.getmtime(target):
        t_tuned = time.strftime("%Y-%m-%d %H:%M",
                                time.localtime(os.path.getmtime(TUNED)))
        t_target = time.strftime("%Y-%m-%d %H:%M",
                                 time.localtime(os.path.getmtime(target)))
        print(f"[apply-tune] 調整結果 ({t_tuned}) が "
              f"{target_rel} ({t_target}) より古いので中止します。")
        print("[apply-tune] そのまま貼ると placement.yaml の変更が巻き戻ります。")
        print("[apply-tune] `make tune TASK=<name>` で取り直してから実行してください。")
        return 1

    tuned_text = open(TUNED).read()
    tuned = yaml.safe_load(tuned_text) or {}
    target_text = open(target).read()
    before = yaml.safe_load(target_text) or {}

    new_text = target_text
    applied = []
    for key in BLOCKS:
        if key not in tuned:
            continue                       # 調整していない項目は触らない
        src = find_block(tuned_text, key)
        dst = find_block(new_text, key)
        if src is None:
            continue
        if dst is None:
            print(f"[apply-tune] WARNING: {target_rel} に {key}: が"
                  f"見つかりません。追記はせず飛ばします。")
            continue
        start, end, _ = dst
        lines = new_text.splitlines(keepends=True)
        new_text = "".join(lines[:start]) + src[2] + "".join(lines[end:])
        applied.append(key)

    if not applied:
        print("[apply-tune] 反映するものがありませんでした。")
        return 1

    # --- 書き込む前に検証する ---------------------------------------
    after = yaml.safe_load(new_text) or {}
    problems = []
    for key in applied:
        if after.get(key) != tuned.get(key):
            problems.append(f"{key}: の値が調整結果と一致しません")
    lost = [k for k in before if k not in after]
    if lost:
        problems.append(f"セクションが消えました: {lost}")
    if problems:
        print("[apply-tune] 検証に失敗したので書き込みを中止します:")
        for p in problems:
            print("  -", p)
        return 1

    backup = target + ".bak"
    shutil.copy2(target, backup)
    with open(target, "w") as f:
        f.write(new_text)

    print(f"[apply-tune] {', '.join(applied)} を {target_rel} に反映しました。")
    print(f"[apply-tune] 元のファイルは {os.path.basename(backup)} に残しています。")
    if "arena_cameras" in applied:
        cams = (after["arena_cameras"] or {}).get("cameras") or {}
        print(f"[apply-tune]   arena_cameras: カメラ {len(cams)} 台")
    return 0


if __name__ == "__main__":
    sys.exit(main())
