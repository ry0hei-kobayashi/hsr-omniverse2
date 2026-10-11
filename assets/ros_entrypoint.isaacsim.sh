#!/bin/bash
# Isaac Sim 5.1 コンテナ用エントリポイント。
# ROS 2 Jazzy は apt ではなく Isaac 同梱物 (+ /app/ros_msgs) を使うので、
# /opt/ros/*/setup.bash は存在しない。必要なのは環境変数だけで、それは
# Dockerfile の ENV (PYTHONPATH / LD_LIBRARY_PATH / ROS_DISTRO) で既に通っている。
set -e

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///cyclonedds.xml}"

# 同梱 rclpy と自前メッセージが見えているかを起動時に 1 行で確認する。
# ここで落ちれば「ROS が見えない」ことが Isaac の起動ログより前に分かる。
/isaac-sim/python.sh - <<'EOF' || { echo "[ros_entrypoint] ROS 2 (Isaac 同梱 jazzy) が import できません" >&2; exit 1; }
import rclpy, control_msgs.action, tmc_control_msgs.action, gazebo_msgs.srv, tf_transformations  # noqa: F401
EOF

exec "$@"
