# System Architecture

## Production GPS Navigation

```text
GPS + compass
      ↓
gps_localization
      ↓
campus waypoint graph / Dijkstra path_planner
      ↓
nav2_route_adapter
      ↓
Nav2: Smac Hybrid-A* + Regulated Pure Pursuit
      ↓
pixhawk_command_adapter
      ↓
MAVROS / Pixhawk / ArduRover
      ↓
Ackermann rover
```

`navigation.launch.py`가 GPS-only Nav2 production entry point다. 이전 custom
GPS navigator는 Git history와 ADR에 보존하며 현재 runtime에는 포함하지 않는다.

## Responsibilities

- `path_planner`: 현재 GPS와 목적지로 campus graph의 Dijkstra route를 선택한다.
- `gps_localization`: MAVROS GPS/compass를 local ENU pose, `/odometry/gps`,
  `map -> odom -> base_link` TF로 변환한다.
- `nav2_route_adapter`: graph route를 Nav2 `NavigateThroughPoses` goal로 변환한다.
- Nav2 Smac Hybrid-A*: heading, non-holonomic motion과 1.7 m 최소회전반경을 고려해
  주행 가능한 path를 계획한다.
- Nav2 RPP: path tracking과 직선/곡선 속도 제어를 담당한다.
- `pixhawk_command_adapter`: Nav2 `/cmd_vel`을 MAVROS `PositionTarget`으로 변환하고
  turning-radius 제한, timeout과 stop 동작을 담당한다.
- Pixhawk/ArduRover: steering/throttle와 actuator safety의 low-level 경계다.
- `turn_radius_calibration`, `speed_calibration`, `steering_pid_recorder`: production
  launch와 분리된 vehicle-control calibration/diagnostic 도구다.
- `gps_waypoint_sampler`: stationary GNSS scatter와 waypoint 좌표를 측정한다.
- `navigation_status_monitor`와 선택적 rosbag recording: route 진행과
  Nav2-to-FCU command chain을 관찰한다.

현재 costmap에는 obstacle source가 없으므로 장애물 회피 기능은 제공하지 않는다.

GPS-only Nav2는 clearance가 충분한 공간의 multi-waypoint 실차 주행까지 검증됐다.
좁은 보행로와 curb 인접 구간에 필요한 정밀 lateral accuracy는 아직 보장하지 않는다.

## Future Extension

```text
GPS + SLAM
     ↓
EKF / localization ─────────→ Nav2
                              ↑
                   LiDAR obstacle/costmap
```

향후 GPS global navigation을 유지하면서 map/sensor-based precise localization을
결합하고 Nav2 costmap에 obstacle source를 추가한다. 이는 방향이며 아직 구현 완료된
architecture가 아니다.
