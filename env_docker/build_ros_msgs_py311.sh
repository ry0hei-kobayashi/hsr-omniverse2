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
source /opt/ros/humble/setup.bash
set -u

export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}

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
colcon build \
    --merge-install \
    --packages-select control_msgs tmc_control_msgs gazebo_msgs tf_transformations \
    --cmake-args \
        -DPython3_EXECUTABLE=$PY \
        -DPYTHON_EXECUTABLE=$PY \
        -DCMAKE_BUILD_TYPE=Release

echo "=== 生成物の ABI を確認 ==="
n311=$(find /ws/install -name '*cpython-311*.so' | wc -l)
n310=$(find /ws/install -name '*cpython-310*.so' | wc -l)
echo "  cpython-311: $n311 本 / cpython-310: $n310 本"
if [ "$n311" -eq 0 ]; then
    echo "FAIL: 3.11 向けの .so が 1 本も出ていない (3.10 で建っている)"; exit 1
fi
if [ "$n310" -gt 0 ]; then
    echo "FAIL: 3.10 向けの .so が混ざっている (Isaac から読めない)"
    find /ws/install -name '*cpython-310*.so' | head; exit 1
fi

echo "=== /out に配る ==="
mkdir -p "$OUT/lib" "$OUT/rclpy"
find /ws/install -name '*.so' -exec cp -f {} "$OUT/lib/" \;
for pkg in control_msgs tmc_control_msgs gazebo_msgs tf_transformations; do
    d=$(find /ws/install -type d -name "$pkg" -path '*site-packages*' | head -1)
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
