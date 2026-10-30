#!/usr/bin/env bash
# Runs inside tmux's own PTY; Ctrl+C reaches the launch process group.
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/ros2_ws/install/gps_navigator/share/gps_navigator/config/runtime_env.sh"
exec "$@"
