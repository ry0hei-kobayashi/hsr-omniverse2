#!/usr/bin/env bash
# Dockerfile.isaacsim の msgbuild ステージ専用。
# /ws/src のメッセージパッケージを Python 3.11 向けに建て、Isaac 同梱 bridge と
# 同じ並べ方 (lib/ と rclpy/) で /out に出す。ホストから直接動かすものではない。
#
# なぜ 3.11 向けに建て直すのか:
#   生成物は 2 種類に分かれる。
#     ABI 非依存 (C):  lib*__rosidl_generator_c.so, lib*__rosidl_typesupport_*.so
#     ABI 依存 (Py):   *_s__rosidl_typesupport_*.cpython-3XX-*.so
#   後者だけ Python の版に縛られる。22.04 の Humble に python3.11 を足し、CMake に
#   Python3_EXECUTABLE を渡して一度で 3.11 向けに出す。生成器 (rosidl_*) は
#   概ね純 Python なので、apt が置いた 3.10 の site-packages を 3.11 に見せる。
set -euo pipefail

SRC=/ws/src
OUT=/out
PY=/py311/bin/python3.11

# ROS の setup.bash は未定義変数を参照するので -u を一時的に外す
set +u
ROS_DISTRO="${ROS_DISTRO:-humble}"
source /opt/ros/$ROS_DISTRO/setup.bash
set -u

# 生成器 (rosidl_*) は apt がその distro の Python (humble: 3.10, jazzy: 3.12) に置いている
export PYTHONPATH=$(ls -d /opt/ros/$ROS_DISTRO/lib/python3.*/site-packages | head -1):${PYTHONPATH:-}

echo "=== 生成器が 3.11 から見えるか ==="
$PY - <<'EOF'
import importlib
missing = []
for m in ('rosidl_adapter', 'rosidl_parser', 'rosidl_cmake',
          'rosidl_generator_c', 'rosidl_generator_py', 'ament_package',
          'em', 'lark'):
    try:
        importlib.import_module(m)
        print(f'  ok   {m}')
    except Exception as exc:
        print(f'  MISS {m}: {exc}')
        missing.append(m)
raise SystemExit(1 if missing else 0)
EOF

echo "=== colcon build (Python 3.11 向け) ==="
cd /ws
# Jazzy の rosidl は FindPython3 を複数回呼ぶので、EXECUTABLE だけでなく ROOT_DIR と
# FIND_STRATEGY=LOCATION でも 3.11 (venv) に固定し、PATH の先頭にも venv を置く
# (humble では EXECUTABLE 指定だけで足りたが、jazzy では system の 3.12 が選ばれた)。
PATH=/py311/bin:$PATH colcon build \
    --merge-install \
    --packages-select control_msgs tmc_control_msgs gazebo_msgs tf_transformations \
    --cmake-args \
        -DPython3_EXECUTABLE=$PY \
        -DPYTHON_EXECUTABLE=$PY \
        -DPython3_ROOT_DIR=/py311 \
        -DPython3_FIND_STRATEGY=LOCATION \
        -DPython3_FIND_VIRTUALENV=ONLY \
        -DCMAKE_BUILD_TYPE=Release

echo "=== 生成物の ABI を確認 ==="
# humble の拡張は *.cpython-310-*.so のように ABI タグ付きだが、jazzy の rosidl_generator_py は
# タグ無し (<pkg>_s__rosidl_typesupport_c.so) で lib/python3.X/site-packages に置く。
# どちらでも通るよう、3.11 の site-packages に拡張があり、かつどの .so も libpython3.12/3.10 を
# 参照していないことで判定する。
SP311=/ws/install/lib/python3.11/site-packages
n311=$(find "$SP311" -name '*.so' 2>/dev/null | wc -l)
# (set -e の下では、最後の grep が不一致で終わると代入全体が失敗扱いになるので || true)
bad=$(for so in $(find /ws/install -name '*.so'); do readelf -d "$so" 2>/dev/null | grep -qE 'libpython3\.(10|12)' && echo "$so"; done || true)
echo "  python3.11 site-packages の拡張: $n311 本"
if [ "$n311" -eq 0 ]; then
    echo "FAIL: 3.11 向けの拡張が 1 本も無い。install 内の .so:"; find /ws/install -name '*.so' | sed 's|.*/||' | sort | head -40
    ls -d /ws/install/lib/python3*/site-packages 2>/dev/null; exit 1
fi
if [ -n "$bad" ]; then
    echo "FAIL: 3.11 以外の libpython を参照する .so がある (Isaac から読めない):"; echo "$bad" | head; exit 1
fi

echo "=== /out に配る ==="
mkdir -p "$OUT/lib" "$OUT/rclpy"
find /ws/install -name '*.so' -exec cp -f {} "$OUT/lib/" \;
for pkg in control_msgs tmc_control_msgs gazebo_msgs tf_transformations; do
    # jazzy は python3.12 の site-packages にも同名の階層を作るので、3.11 側を優先して探す
    d=$(find "$SP311" -maxdepth 1 -type d -name "$pkg" | head -1)
    [ -z "$d" ] && d=$(find /ws/install -type d -name "$pkg" -path '*site-packages*' | head -1)
    if [ -z "$d" ]; then
        echo "FAIL: $pkg の Python パッケージが見つからない"; exit 1
    fi
    cp -rf "$d" "$OUT/rclpy/"
done
# 純 Python パッケージ (tf_transformations) は 3.10 の site-packages に入るので、
# 3.10 でコンパイルされた __pycache__ は配らない (3.11 側で作り直される)。
find "$OUT/rclpy" -name __pycache__ -type d -prune -exec rm -rf {} +
# tmc_wrs_gazebo_worlds は CMake 側が xacro/tf2 を要求する (world 生成用) が、
# こちらで使うのは純 Python の randomizer だけなので、colcon を通さず
# Python パッケージをそのままコピーする (依存は numpy / scipy のみ)。
cp -rf "$SRC/tmc_wrs_gazebo/tmc_wrs_gazebo_worlds/tmc_wrs_gazebo_worlds" "$OUT/rclpy/"
# ライセンス表記を一緒に配る
mkdir -p "$OUT/licenses"
for p in "$SRC"/*; do
    n=$(basename "$p")
    for f in LICENSE LICENSE.txt LICENSE.md; do
        [ -f "$p/$f" ] && cp -f "$p/$f" "$OUT/licenses/$n.LICENSE" && break
    done
done

echo "=== 配ったもの ==="
echo "  lib/   : $(ls "$OUT/lib" | wc -l) 本"
echo "  rclpy/ : $(ls "$OUT/rclpy" | tr '\n' ' ')"
echo "done"
