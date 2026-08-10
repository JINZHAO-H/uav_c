#!/usr/bin/env bash
set -euo pipefail

colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
