# 실행 안내

모든 명령은 로버의 Jetson에서 실행한다. 먼저 [빌드](../README.md#시작하기)를 마치고,
센서 연결·TF·정지 수단을 확인한다. mapping/validation/drive는 동시에 실행하지 않는다.

## 1. 터미널 환경과 지도 폴더

새 터미널마다 실행한다. 아래 지도 이름은 예시이며 본인의 폴더로 바꾼다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/install/rover_navigation/share/rover_navigation/config/runtime_env.sh
export MAP_DIR="$HOME/ros2_ws/maps/test_map/map1"
roskeep() { bash "$HOME/ros2_ws/src/tools/ros_session.sh" "$@"; }
```

`MAP_DIR`은 지도 관련 명령에만 필요하다. 기본 DDS 설정은 Jetson 내부 통신용이다.

## 2. 센서 실행 방식

센서를 각각 실행하려면 아래 명령을 사용한다. 카메라 런치가 장착 TF도 함께 발행하므로
별도의 `static_transform_publisher`를 중복 실행하지 않는다.

```bash
roskeep start mavros ros2 launch rover_navigation mavros.launch.py \
  fcu_url:=serial:///dev/ttyACM0:115200
roskeep start camera ros2 launch rover_navigation camera.launch.py
roskeep start lidar ros2 launch rover_navigation lidar.launch.py
```

기존 지도 주행·이어 촬영은 camera 명령 대신 아래 명령으로 지도별 카메라 설정을 사용한다.

```bash
roskeep start camera ros2 launch rover_navigation camera.launch.py \
  runtime_file:="$MAP_DIR/localization.yaml"
```

또는 센서를 따로 실행하지 않고 아래 통합 명령의 `start_mavros:=true start_camera:=true
start_lidar:=true`를 사용한다. 이미 센서가 켜져 있다면 해당 인자를 생략한다(기본 false).
mapping과 navigation은 동시에 실행하지 않으며, 웹에서 준비한 navigation과도 중복 실행하지 않는다.

## 3. 새 지도·이어 촬영

VNC 데스크톱 터미널에서 실행하면 RTAB-Map 화면도 함께 열린다. 새 지도는 비어 있는
`MAP_DIR`을 사용한다. 다음 예시는 센서까지 통합 실행한다.

카메라 전용 새 지도:

```bash
roskeep start mapping ros2 launch rover_navigation mapping.launch.py \
  map_dir:="$MAP_DIR" sensors:=rgbd rtabmap_viz:=true \
  start_mavros:=true start_camera:=true
```

카메라·라이다 융합 새 지도:

```bash
roskeep start mapping ros2 launch rover_navigation mapping.launch.py \
  map_dir:="$MAP_DIR" sensors:=rgbd_lidar rtabmap_viz:=true \
  start_mavros:=true start_camera:=true start_lidar:=true
```

이어 촬영은 기존 `rtabmap.db`와 `localization.yaml`이 있는 폴더를 지정한다.
`sensors:=auto`가 저장된 센서 구성을 유지한다. 기존 구간에서 정합된 것을 확인하고 이동한다.

```bash
roskeep start mapping ros2 launch rover_navigation mapping.launch.py \
  map_dir:="$MAP_DIR" continue:=true rtabmap_viz:=true \
  start_mavros:=true start_camera:=true start_lidar:=true
```

카메라 전용 지도이며 라이다가 없다면 `start_lidar:=true`를 생략한다.
카메라 전용 DB를 이어 촬영하면서 융합 지도로 바꾸지 않는다. 센서 구성을 바꾸려면 새 지도를 만든다.
원본을 보존하며 이어 촬영하려면 mapping 정상 종료 뒤 지도 폴더를 복사하고 새 경로를 지정한다.

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

아래는 VNC 터미널에서 센서까지 통합 실행하는 예시다. 센서를 별도로 실행했다면 `start_*:=true`를 생략한다.

```bash
roskeep start navigation ros2 launch rover_navigation navigation.launch.py \
  mode:=validation map_dir:="$MAP_DIR" rviz:=true \
  start_mavros:=true start_camera:=true start_lidar:=true lidar_obstacles:=true
```

```bash
ros2 topic echo /navigation/ready std_msgs/msg/Bool --once
ros2 topic echo /navigation/status std_msgs/msg/String --qos-durability transient_local --once
ros2 run tf2_ros tf2_echo map base_link
```

필요하면 RViz 2D Pose Estimate를 입력한다. TF 이동만으로 정합 성공은 아니다.
`ready=true`와 실제 위치·방향, 관측한 구조물의 지도 일치를 확인한다.
validation은 차량 어댑터가 없어 이동하지 않는다. 먼 목표를 오래 유지하면 progress 검사로 실패할 수 있다.

`roskeep stop navigation`으로 validation을 정상 종료한 뒤 실차 주행을 실행한다.

```bash
roskeep start navigation ros2 launch rover_navigation navigation.launch.py \
  mode:=drive map_dir:="$MAP_DIR" rviz:=true \
  start_mavros:=true start_camera:=true start_lidar:=true lidar_obstacles:=true
```

기본 `local_odometry:=auto`는 지도에 저장된 센서 구성을 따른다. 카메라 전용 `map1`에서도
라이다 회피는 위 명령으로 켜지지만, EKF까지 시험하려면 `local_odometry:=fused`를 추가한다.
라이다 없이 카메라만 사용하려면 `start_lidar:=true`를 빼고 `lidar_obstacles:=false`로 실행한다.

정합·ready 상태를 확인하고 운영자가 GUIDED/아밍과 정지 수단을 준비한 뒤 Nav2 Goal을 지정한다.
직선 목표는 0.35m/s, 커브는 0.25~0.35m/s, 명령 상한은 0.4m/s다.
FCU 실제 속도와 방향이 명령을 따르는지는 별도로 확인한다.
Goal이 aborted여도 노드가 정상이고 ready라면 launch 재시작 없이 새 Goal을 지정할 수 있다.
경로 계획의 시간 제한은 해제돼 있지만, 경로 부재·차량 제약·추적 실패 등으로 실패할 수 있다.

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
roskeep stop lidar
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

### 홈 전체를 SSD로 옮기기

기존 구성은 `~/ssd`에 NVMe를 마운트하고 `~/ros2_ws`, `~/.ros`, pip 캐시만 연결한다.
이전 후에는 NVMe를 `/home/ykk` 자체에 마운트한다. OS·`/opt/ros`는 SD에 남고,
프로젝트·설정·SSH 키·VS Code·로그·사용자 파일은 SSD에 저장된다.
`ssd/` 경유 경로나 사용자 홈 사본은 남기지 않는다.
다른 이름의 개인 프로젝트/가상환경은 동일한 사본이라는 근거 없이 지우지 않는다.

차량 시동을 해제하고 진행한다. 다음 명령은 관리자 이전 작업을 **다음 부팅에 예약**한다.
디스크를 포맷하거나 지금 로그인 세션의 홈을 옮기지는 않는다.

```bash
sudo python3 ~/ros2_ws/src/tools/storage/migrate_home.py install
# 작업을 저장한 뒤 재부팅. 이전 중에는 SSH/웹 접속이 늦게 열린다.
sudo reboot
```

부팅 시 사용자 프로세스가 시작되기 전에 파일을 복사하고 checksum으로 확인한다.
기존 SSD 작업 공간은 재복사하지 않는다. 지도 DB·출발점 식별 경로와 캐시 키를 갱신하고
새 경로에서 ROS를 다시 빌드한 뒤에만 SD 홈 원본을 제거한다.
검증 또는 재빌드 실패 시 SD 홈과 기존 마운트로 복구하며, 성공하지 않은 상태를 완료로 표시하지 않는다.
기존 기체 설정 백업은 파라미터 복구 자료이므로 일반 홈 사본과 구분해 유지한다.

완료 확인:

```bash
findmnt -T /home/ykk
findmnt -T /home/ykk/ros2_ws
journalctl -u campus-home-migration.service -b --no-pager
```

두 경로가 모두 NVMe를 가리켜야 한다. 서비스의 SSD 검사는 기존 분리 마운트와
전체 홈 마운트를 모두 지원하며, SSD가 없는 상태에서 SD에 새 작업 공간을 만들지 않는다.

홈 마운트와 ROS 재빌드가 완료된 뒤 로그에
`Unexpected source device; refusing SD cleanup`이 남으면 이전 설치본의 shared mount
전파 때문에 마지막 SD 정리만 중단된 상태다. 다음 명령은 완료 기록·SSD UUID·fstab을
검증하고, 별도 mount namespace에서 SD의 숨겨진 옛 홈만 제거한다. 실행 중인 SSD 홈과
서비스를 다시 옮기거나 재시작하지 않는다. 성공한 경우에만 이전 서비스의 실패 표시를 해제한다.

```bash
sudo python3 ~/ros2_ws/src/tools/storage/migrate_home.py cleanup
```

## 7. 기존 GPS 경로와 차량 시험 도구

RTAB 주행과 동시에 실행하지 않는다. 검토된 GPS waypoint 그래프를 사용하는 기존 방식:

```bash
ros2 launch rover_navigation gps_navigation.launch.py destination:=W05
```

별도 시험 실행점은 `turn_radius_calibration`, `speed_calibration`,
`steering_pid_recorder`, `gps_waypoint_sampler`다. 차량을 움직이는 시험은 Nav2·다른 명령
발행자를 종료하고 모드·아밍·중단 수단을 준비한 뒤 실행한다.
