#!/usr/bin/env bash
# 为 clangd 生成 compile_commands.json
set -euo pipefail

CATKIN_WS="$(cd "$(dirname "$0")/../../.." && pwd)"
source /opt/ros/noetic/setup.bash

cd "$CATKIN_WS"
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="avp" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 确保包根目录的符号链接存在
ln -sf "$CATKIN_WS/build/compile_commands.json" \
  "$CATKIN_WS/src/AVP_SLAM_ROS/compile_commands.json"

echo "Done: $CATKIN_WS/build/compile_commands.json"
echo "Symlink: $CATKIN_WS/src/AVP_SLAM_ROS/compile_commands.json"
