#!/usr/bin/env bash
set -eo pipefail
app_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/ros2_ws/install/rover_navigation/share/rover_navigation/config/runtime_env.sh"
set -a
source "$HOME/.config/campus-rover/mission.env"
set +a
set -u
export PYTHONPATH="$app_root${PYTHONPATH:+:$PYTHONPATH}"
mission_map_dir="${CAMPUS_MAP_DIR:-$HOME/ros2_ws/maps/test_map/map1}"
case "${1:-}" in
  web)
    exec python3 -m campus_mission.app --mode live --service-controls \
      --map-dir "$mission_map_dir" --host "${CAMPUS_WEB_HOST:-0.0.0.0}" \
      --port "${CAMPUS_WEB_PORT:-8080}"
    ;;
  navigation)
    exec ros2 launch rover_navigation navigation.launch.py mode:=drive \
      map_dir:="$mission_map_dir" rviz:="${CAMPUS_RVIZ:-false}" \
      local_odometry:="${CAMPUS_NAV_ODOMETRY:-auto}" \
      lidar_obstacles:="${CAMPUS_LIDAR_OBSTACLES:-true}" \
      start_mavros:=true start_camera:=true start_lidar:=true
    ;;
  *) echo 'Usage: run_service.sh web|navigation' >&2; exit 2 ;;
esac
