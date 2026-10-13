#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/ws/install/local_setup.bash" --

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///cyclonedds.xml}"

exec "$@"
