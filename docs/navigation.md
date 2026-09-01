# Navigation

## Current Configuration

- Global route: campus GPS waypoint graph and Dijkstra
- Localization: GPS/compass local ENU pose and `map -> odom -> base_link`
- Planner: Nav2 Smac Hybrid-A*, Dubin model, minimum turning radius 1.7 m
- Controller: Nav2 Regulated Pure Pursuit, straight 0.60 m/s, curve target 0.40 m/s
- Vehicle interface: Nav2 `/cmd_vel` to MAVROS/Pixhawk `PositionTarget`
- Obstacles: no sensor or costmap obstacle layer yet

## Build and Run

MAVROS를 먼저 실행하고 FCU 연결을 확인한다.

```bash
cd /home/ykk/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gps_navigator --symlink-install
source /home/ykk/ros2_ws/install/setup.bash

ros2 launch gps_navigator navigation.launch.py destination:=W05
```

`navigation.launch.py`는 GPS-only Nav2 production entry point다. 이전 custom GPS
navigator는 Git history와 ADR에 보존하며 현재 runtime에는 포함하지 않는다.

## Before Hardware Operation

- FCU mode/armed 상태, emergency stop과 수동 중단 방법을 확인한다.
- GPS/compass freshness와 ENU position/yaw 방향을 확인한다.
- `map -> odom -> base_link` TF를 확인한다.
- `/mavros/setpoint_raw/local` command publisher가 하나인지 확인한다.
- watchdog이 stale command에서 speed와 yaw-rate를 0으로 만드는지 확인한다.
- 장애물이 없는 통제된 공간에서 직선, 좌회전, 우회전을 순서대로 검증한다.

현재 Nav2 구성은 넓고 clearance가 충분한 공간의 GPS-based multi-waypoint 주행까지
실차 검증됐다. GPS-only localization은 좁은 보행로와 curb 인접 구간의 정밀
lateral safety를 보장하지 않으며, 현재 obstacle avoidance도 제공하지 않는다.
