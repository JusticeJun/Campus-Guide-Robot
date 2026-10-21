# 실행 안내

모든 명령은 로버의 Jetson에서 실행한다. 먼저 [빌드](../README.md#시작하기)를 마치고,
센서 연결·TF·정지 수단을 확인한다. mapping/validation/drive는 동시에 실행하지 않는다.

## 1. 터미널 환경과 지도 폴더

새 터미널마다 실행한다. 아래 지도 이름은 예시이며 본인의 폴더로 바꾼다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/install/gps_navigator/share/gps_navigator/config/runtime_env.sh
export MAP_DIR="$HOME/ros2_ws/maps/test_map/map1"
roskeep() { bash "$HOME/ros2_ws/src/tools/ros_session.sh" "$@"; }
```

`MAP_DIR`은 지도 관련 명령에만 필요하다. 기본 DDS 설정은 Jetson 내부 통신용이다.

## 2. 센서 실행

```bash
roskeep start mavros ros2 launch gps_navigator mavros.launch.py \
  fcu_url:=serial:///dev/ttyACM0:115200
roskeep start camera ros2 launch gps_navigator camera.launch.py
roskeep start camera_tf ros2 run tf2_ros static_transform_publisher \
  --x 0.3001 --y 0.0175 --z 0.2032 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id camera_link
```

기존 지도 주행·이어 촬영은 camera 명령 대신 아래 명령을 사용한다. 같은 드라이버를 중복 실행하지 않는다.

```bash
roskeep start camera ros2 launch gps_navigator camera.launch.py \
  runtime_file:="$MAP_DIR/localization.yaml"
```

## 3. 새 지도·이어 촬영

새 지도는 사용하지 않은 `MAP_DIR`에서 실행한다.

```bash
roskeep start mapping ros2 launch gps_navigator mapping.launch.py \
  map_dir:="$MAP_DIR" rtabmap_viz:=false
```

이어 촬영은 원본 `rtabmap.db`와 `localization.yaml`이 있는 폴더를 지정한다.
기존 구간에서 정합된 것을 확인하고 새 구간으로 이동한다.

```bash
roskeep start mapping ros2 launch gps_navigator mapping.launch.py \
  map_dir:="$MAP_DIR" continue:=true rtabmap_viz:=false
```

원본을 보존하면서 이어 촬영하려면 mapping이 종료된 뒤 지도 폴더 전체를 새 폴더에 복사하고
새 경로를 `MAP_DIR`로 지정한다. 작동 중인 DB를 단순 복사하지 않는다.

VNC 데스크톱 터미널에서 화면만 별도로 실행한다.

```bash
roskeep start viz ros2 run rtabmap_viz rtabmap_viz --ros-args \
  -r __ns:=/rtabmap -p subscribe_rgbd:=true -p subscribe_depth:=false \
  -p frame_id:=base_link -p approx_sync:=true -p qos_image:=1 -p qos_odom:=1 \
  -r rgbd_image:=/rtabmap/odom_rgbd_image -r odom:=/odom
```

## 4. 지도 저장

차량을 멈추고 mapping이 켜진 상태에서 2D 지도를 저장한다. 이어 촬영 후에도 다시 내보낸다.

```bash
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/rtabmap" --ros-args \
  -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0 -r map:=/rtabmap/map
roskeep stop mapping
roskeep stop viz
```

DB 저장이 끝나 정상 종료될 때까지 기다린다. 다음 네 파일을 함께 유지한다.

```text
map1/
├── rtabmap.db
├── localization.yaml
├── rtabmap.yaml
└── rtabmap.pgm
```

## 5. 위치추정 확인과 주행

센서·카메라 TF는 계속 실행한 상태에서 VNC 터미널로 실행한다.

```bash
ros2 launch gps_navigator navigation.launch.py \
  mode:=validation map_dir:="$MAP_DIR" \
  start_mavros:=false start_camera:=false start_camera_tf:=false rviz:=true
```

```bash
ros2 topic echo /navigation/ready std_msgs/msg/Bool --once
ros2 topic echo /navigation/status std_msgs/msg/String --qos-durability transient_local --once
ros2 run tf2_ros tf2_echo map base_link
```

필요하면 RViz 2D Pose Estimate를 입력한다. TF 이동만으로 정합 성공은 아니다.
`ready=true`와 실제 위치·방향, 관측한 구조물의 지도 일치를 확인한다.
validation은 차량 어댑터가 없어 이동하지 않는다. 먼 목표를 오래 유지하면 progress 검사로 실패할 수 있다.

validation을 종료한 뒤 실차 주행을 실행한다.

```bash
ros2 launch gps_navigator navigation.launch.py \
  mode:=drive map_dir:="$MAP_DIR" \
  start_mavros:=false start_camera:=false start_camera_tf:=false rviz:=true
```

정합·ready 상태를 확인하고 운영자가 GUIDED/아밍과 정지 수단을 준비한 뒤 Nav2 Goal을 지정한다.
직선 목표는 0.35m/s, 커브는 0.25~0.35m/s, 명령 상한은 0.4m/s다.
FCU 실제 속도와 방향이 명령을 따르는지는 별도로 확인한다.
Goal이 aborted여도 노드가 정상이고 ready라면 launch 재시작 없이 새 Goal을 지정할 수 있다.
경로 계획의 시간 제한은 해제돼 있지만, 경로 부재·탐색 횟수 한도 등으로 실패할 수 있다.

추적 손실로 정지가 래치되면 실제 정지·FCU HOLD·위치추정 회복·기존 Goal 취소를 확인한 뒤 해제한다.

```bash
ros2 service call /navigation/acknowledge_tracking_stop std_srvs/srv/Trigger '{}'
```

서비스 성공과 `ready=true`를 확인하고 GUIDED 전환 후 새 Goal을 지정한다.
이 서비스는 아밍이나 자동 출발 명령이 아니다. 추적 손실 래치에만 적용되며,
운영자 정지 등 다른 영구 정지 원인은 원인을 해결한 뒤 launch를 다시 시작해야 한다.

## 6. 접속 복구·종료·저장 공간

```bash
roskeep status
roskeep attach mapping
```

로그에서 **Ctrl+B, D**는 분리만 하고 계속 실행한다. **Ctrl+C**는 실제 노드를 종료한다.
SSH/VNC 연결이 끊겨도 세션은 유지되지만 재부팅·전원 차단에는 적용되지 않는다.
종료할 때는 navigation/mapping을 먼저 정상 종료하고 센서를 종료한다.

```bash
roskeep stop camera
roskeep stop mavros
roskeep stop camera_tf
```

| 경로 (`~/ros2_ws` 아래) | 내용 / 정리 기준 |
|---|---|
| `src/` | Git 소스와 문서 |
| `maps/` | 원본 지도; 자동 생성 가능 파일로 취급하지 않음 |
| `.cache/localization/` | 재사용 DB; navigation 종료 후 삭제 가능, 다음 시작이 느려짐 |
| `log/navigation/` | 실행 설정과 크기가 제한된 주행 추적; 종료 후 정리 가능 |
| `build/`, `install/` | 빌드·실행 파일; 특히 symlink-install은 build도 필요 |
| `log/build_*`, `log/test_*` | 빌드·테스트 기록 |

영상/rosbag은 기본적으로 녹화하지 않는다. `record_bag:=true`, `record_camera:=true`는
필요할 때만 사용하며 용량을 따로 관리한다.

## 7. 기존 GPS 경로와 차량 시험 도구

RTAB 주행과 동시에 실행하지 않는다. 검토된 GPS waypoint 그래프를 사용하는 기존 방식:

```bash
ros2 launch gps_navigator gps_navigation.launch.py destination:=W05
```

별도 시험 실행점은 `turn_radius_calibration`, `speed_calibration`,
`steering_pid_recorder`, `gps_waypoint_sampler`다. 차량을 움직이는 시험은 Nav2·다른 명령
발행자를 종료하고 모드·아밍·중단 수단을 준비한 뒤 실행한다.
