# Campus Guide Robot

ROS 2 Humble과 NVIDIA Jetson Orin Nano에서 실행되며 MAVROS를 통해
Pixhawk 2.4.8/ArduRover 기반 Ackermann rover를 제어하는 실외 자율주행 프로젝트다.

## Architecture

```text
M8N GPS + compass
  -> GPS local ENU localization
  -> Campus waypoint graph / Dijkstra route
  -> Nav2 / Smac Hybrid-A*
  -> Regulated Pure Pursuit
  -> Velocity Smoother
  -> Pixhawk Command Adapter
  -> MAVROS -> Pixhawk / ArduRover -> Ackermann rover
```

Smac Hybrid-A*와 command adapter는 동일한 1.7 m minimum turning radius를
사용한다. 현재 costmap에는 obstacle source가 없으며, GPS-only navigation은 넓고
clearance가 충분한 공간의 multi-waypoint 실차 주행까지 검증됐다. 좁은 구간의
정밀 주행에는 향후 map/sensor-based localization을 결합한다.

설계 결정은 [ADR](docs/adr/README.md), node 책임과 데이터 흐름은
[architecture](docs/architecture.md), 차량 제약은 [hardware](docs/hardware.md)에 있다.

## Build and Run

MAVROS와 FCU 연결을 먼저 준비한다.

```bash
cd <ros2-workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select gps_navigator --symlink-install
source install/setup.bash
ros2 launch gps_navigator navigation.launch.py destination:=W05
```

실차 실행 전 FCU mode/armed 상태, emergency stop, GPS/compass freshness,
TF, 단일 command publisher와 watchdog을 확인한다. 상세 확인 항목은
[navigation guide](docs/navigation.md)를 따른다.

## Utilities

Production launch와 분리된 `turn_radius_calibration`, `speed_calibration`,
`steering_pid_recorder`, `gps_waypoint_sampler`를 차량 및 GNSS 특성 검증에 사용한다.
이 도구들은 production navigation과 동시에 차량 command를 발행하지 않는다.
