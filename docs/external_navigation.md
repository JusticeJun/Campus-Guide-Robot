# GPS·나침반 없이 Pixhawk를 사용하는 구성

상태: 2026-10-22 Pixhawk 6C/ArduRover 4.7.1에 설정을 적용하고 전원 재연결 후 유지와
정지 상태의 실제 EKF 융합을 확인했다. 전체 변경 전 값은 백업했고 PWM·속도·조향·시동 검사
설정은 해당 변경에서 유지했다. 이후 조향 변경값은 [하드웨어](hardware.md)를 따른다.
최신 융합 구성의 실외 이동·입력 상실 검증은 별도다.
`navigation.launch.py`의 기본은 `fcu_estimation:=externalnav`다.
`validation`도 기본적으로 추정값을 FCU에 전송하지만 차량 명령 어댑터는 실행하지 않는다.
GPS 모듈을 빼거나 FCU 설정을 바꾸는 작업은 launch가 자동으로 하지 않는다.

## 선택한 구조와 근거

```text
RealSense RGB-D + 카메라 IMU → rgbd_odometry /odom
                              ├─ RTAB-Map → map→odom → Nav2
                              └─ external_navigation → MAVROS → Pixhawk EKF3
Nav2 → 기존 guard/adapter → 목표 전진 속도·yaw rate → Pixhawk → ESC/조향 PWM
```

카메라+라이다 모드에서는 위 `/odom`을 [로컬 EKF 융합 출력](sensor_fusion.md)으로 대체한다.
카메라 하나의 추적 실패는 ExternalNav 중단 조건이 아니며, 독립 라이다 추적이 이어지면 계속 전송한다.
두 추적 소스가 모두 실패한 경우는 별도로 중단한다. Pixhawk raw gyro만 로컬 EKF로 입력하고
FCU에서 이미 융합한 ExternalNav 속도·yaw를 다시 입력하지 않는다.

카메라의 **연속적인 `/odom`**을 Pixhawk의 위치·속도·yaw 입력으로 쓴다.
지도 재정합 때 이동할 수 있는 `map→base_link`는 FCU 입력으로 쓰지 않는다.
Pixhawk의 IMU, 속도·yaw-rate 제어기와 검증된 조향 FF/P는 유지한다.
기존 DB와 Nav2의 직선 0.35m/s, 커브 0.25~0.35m/s, 상한 0.4m/s도 유지한다.
카메라의 단절 없는 움직임 추정과 기존 지도에서의 장소 인식은 서로 다른 기능이다.
초기 지도 정합 후에는 연속 VO로 주행하며, 중간 지도 정합이 성공하면 RTAB이 보정한다.
매 프레임 지도 정합 성공을 요구하지 않는다. 여기서 추적 실패는 `rgbd_odometry`의
`OdomInfo.lost`이며, RTAB 장소 인식/loop closure가 발생하지 않은 프레임을 뜻하지 않는다.

| 참고한 구현 | 적용할 점 | 그대로 성능을 보장하지 않는 이유 |
|---|---|---|
| [ArduPilot T265 VIO 연동](https://ardupilot.org/dev/docs/ros-vio-tracking-camera.html), [구현자의 동작 사례](https://discuss.ardupilot.org/t/easier-setup-for-intel-realsense-t265/53991) | GPS 없는 외부 위치추정과 FCU 제어의 분리 | 전용 VIO 카메라 T265이며 우리 D435if+RTAB 연산 지연과 다름 |
| [Jetson TX2·RPLIDAR A2·Cartographer 연동](https://ardupilot.org/dev/docs/ros-cartographer-slam.html) | ROS의 SLAM 결과를 MAVROS로 FCU에 공급 | ROS 1 예제이며 문서도 개발 중인 구성을 명시. 우리 전방 라이다 범위·실외 성능의 검증 자료는 아님 |
| [ArduPilot 공식 ExternalNav 인터페이스](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html) | ODOMETRY, 실제 갱신 4Hz 이상, EKF origin과 source 설정 | 인터페이스 지원과 우리 기체의 현장 성공은 별개 |

따라서 지금은 Pixhawk를 PWM 출력기로 바꾸거나 별도 제어기를 만드는 대신,
이미 동작하는 하위 제어기에 올바른 센서 입력을 공급한다.
새 라이다의 지도 정합·장애물 회피는 이 변경과 독립적으로 추가한다.

## 좌표·전송과 보호 범위

- `/odom`: 부모 `odom`, 자식 `base_link`, 속도는 body FLU. `map` 입력은 거부한다.
- 사용자 측정 Pixhawk 본체 중앙: `base_link`에서 `[0.25, 0, 0.07]m`, 화살표 전방,
  차체와 평행. 본체 중앙은 IMU 칩 위치의 근사이며 정밀 외부 파라미터 보정값은 아니다.
- `p_fcu = p_base + R*r`, `v_fcu = v_base + ω×r`로 회전 시 장착 위치 차이를 보정한다.
  따라서 FCU의 `VISO_POS_X/Y/Z`는 0이며 카메라 장착 좌표를 다시 넣지 않는다.
- FLU→FRD 변환 후 `LOCAL_FRD(20)/BODY_FRD(12)`로 보낸다. 지역축은 진북 정렬이 아니다.
  QGC의 방향을 진북 나침반으로 사용하거나 GPS waypoint/RTL 주행을 혼용하지 않는다.
- MAVROS ROS 2 router의 `/uas1/mavlink_sink`로 MAVLink 2 ODOMETRY를 전달한다.
  별도 시리얼 연결이나 두 번째 모터 명령 publisher를 만들지 않는다.
  이 경로는 실제 MAVROS UAS prefix가 바뀌면 설정도 바꿔야 한다.
- `reset_counter`는 브리지 재시작마다 영속 파일에서 증가한다. 첫 전송 전에 disarmed 상태를
  실제로 관측해야 한다. 동시 브리지는 파일 잠금으로 막는다.
- 원래 측정 timestamp를 보존한다. 현재 실차 프로필은 측정 지연 1.0초를 넘긴
  개별 샘플만 버리고, 유효 입력이 2초 넘게 끊기면 실패로 처리한다. 지연된 한 샘플이
  이전 정상 입력 이력까지 지우지는 않는다. `OdomInfo.lost` 역시 마지막 정상 수신부터
  2초 이내이면 잘못된 샘플만 전송하지 않고 정상 입력의 복귀를 기다린다. lost 알림은
  이 시간 제한을 연장하지 않는다. 좌표/시간 리셋, NaN·비정상 covariance는 즉시
  실패로 처리한다. 오래된 자료를 반복 전송하지 않는다.
- 유효한 측정값 전송과 주행 준비 판정을 분리한다. 시동 해제 상태에서 FCU 설정을
  확인하면 첫 유효 샘플부터 전송하며, 시작 시 일시적인 오류가 있어도 다음 정상 샘플은
  전달한다. 시동 중 실패나 좌표 리셋을 이 경로로 해제하지 않는다.
- 현재 실차 프로필의 초기 주행 준비는 실제 입력 2Hz 이상을 2초 확인하고 FCU 융합 상태도
  별도로 확인한다. 한 번 준비를 통과하면 측정 신선도·입력 간격·FCU 추정이 정상인 동안
  평균 수신률이나 시작 대기 조건을 다시 강제하지 않는다. 이 범위의 도착 간격 변동은
  목표 취소·zero 명령·HOLD 전환 없이 현재 Nav2 명령을 계속 전달한다.
  실제 입력/FCU 실패는 준비를 해제하며, 회복 뒤 초기 준비 조건을 다시 확인한다.
  `minimum_ready_rate_hz`, `max_sample_age_s`, `max_odometry_gap_s`는
  `config/external_navigation.yaml`에 있다. RTAB의 `lost` 판정에 더해 임의의 특징점
  개수 기준을 다시 강제하지 않는다.
  ArduPilot 공식 안내는 4Hz 이상이며, 2Hz 허용은 이 로버의 현장 검증용 설정이다.
  정지 상태에서 FCU 융합이 된다는 사실이 이동 중 추정 정확도를 보장하지 않는다.
- FCU source 설정, EKF 자세·수평 속도·위치 상태, 최신 FCU body 속도 수신을 확인한다.
  ArduRover의 `EKF_STATUS_REPORT(193)`를 `/uas1/mavlink_source`에서 읽는다.
  MAVROS 2.12의 `/mavros/estimator_status`는 다른 메시지 `ESTIMATOR_STATUS(230)`용이므로
  이 토픽을 ArduRover 준비 판정에 사용하지 않는다.
  카메라와 FCU의 속도 차이는 진단값으로만 제공하며 작은 차이로 주행을 차단하지 않는다.
- `quality=1`은 유효 샘플 표시이지 정확도 1%라는 뜻이나 보정된 신뢰도 점수가 아니다.
  covariance에 최소 분산을 더하며 RTAB 분산이 실제 오차를 완벽히 표현한다고 가정하지 않는다.
- `/external_navigation/ready`는 지도 정합 상태와 별도다. guard와 차량 adapter가 함께 확인하며
  마지막 정상 heartbeat 이후 2초까지의 수신 지연은 목표 취소·HOLD 없이 허용한다.
  2초를 넘긴 상태 메시지 단절이나 명시적인 `ready=false`는 명령을 차단한다.
  이 시간은 VO 입력 간격 검사와 별개이며, Nav2 명령 자체의 0.5초 만료는 유지한다.
  진행 중 실패는 기존 HOLD/정지 경로를 사용한다. 입력 단절 후 좌표계를 재설정해 자동 재출발하지 않는다.
  시동 전의 일시적 입력 단절은 정상 입력·정지·FCU 상태 회복 후 자동 재시도한다.
  시동 중 발생한 실패는 아래 복구 서비스로 재시도하며 전체 launch/DB 로딩을 반복하지 않는다.
- 시동 체크를 끄지 않는다. FCU EKF failsafe는 HOLD를 유지한다. Jetson 전원/통신 단절 때의
  FCU 자체 정지도 최초 실차 검증에 포함한다. ROS의 zero/HOLD 요청만으로 물리 정지를 보장하지 않는다.

## 적용 순서

### 1. 기존 설정 그대로 미리보기

MAVROS·카메라·카메라 TF는 기존 명령으로 실행한다. 주행 launch와 mapping을 중복 실행하지 않는다.

```bash
ros2 launch rover_navigation navigation.launch.py \
  mode:=validation map_dir:="$MAP_DIR" fcu_estimation:=externalnav externalnav_transmit:=false \
  start_mavros:=false start_camera:=false start_camera_tf:=false rviz:=true
```

`externalnav_transmit:=false`를 명시한 이 명령은 preview다. `/external_navigation/preview`에만 ODOMETRY를 발행하고
FCU에는 보내지 않는다. `/external_navigation/status`의 `input_ready`가 true인지 확인한다.
preview의 `ready=false`는 정상이다. FCU 설정이 GPS여서 audit 불일치가 나오는 것도 이 단계에서는 정상이다.

```bash
ros2 topic echo /external_navigation/status
```

### 2. 정지·disarm 상태에서 FCU 설정 변경

1. QGC Parameters의 Tools에서 **현재 전체 파라미터를 파일로 저장**한다.
   기존 GPS 모드 복원에는 이 파일을 사용한다. 저장하지 않고 기본값 복원을 하지 않는다.
2. `VISO_TYPE=3`을 설정하고 재부팅하여 VISO 설정 항목을 활성화한다.
3. `rover_navigation/config/externalnav_rover47.param`을 QGC에서 불러오고 변경 목록을 확인·적용한다.
   대상은 확인한 ArduRover 4.7.1, system/component 1/1이다. 다른 펌웨어/기체에는 그대로 적용하지 않는다.
4. 재부팅한다. 주요값은 `EK3_SRC1_POSXY=6`, `VELXY=6`, `YAW=6`, `POSZ=1`, `VELZ=0`,
   `GPS1_TYPE=GPS2_TYPE=0`, `COMPASS_USE/USE2/USE3=0`이다. IMU 사용과 제어 게인은 유지한다.
5. source 선택은 set 1로 유지한다. RC 또는 스크립트의 EKF source switching을 혼용하지 않는다.
   source 2/3의 수평 위치·속도·yaw는 이 프로파일에서 비활성화한다.
6. 아래 validation 전송 시험을 먼저 수행한다. 시동 체크를 우회하거나 force-arm하지 않는다.

```bash
ros2 launch rover_navigation navigation.launch.py \
  mode:=validation map_dir:="$MAP_DIR" fcu_estimation:=externalnav \
  externalnav_transmit:=true \
  start_mavros:=false start_camera:=false start_camera_tf:=false rviz:=true
```

GPS 없는 EKF는 **origin**이 필요하다. origin은 캠퍼스 부근의 지리적 기준점이며 GPS 센서나
RViz initial pose를 대신하는 값이 아니다. 매번 실제 출발 좌표를 측량하는 입력도 아니다.
이 기체에는 기존 `waypoints.yaml`의 누리관 좌표 35.134843, 129.103800을 저장하고,
ArduRover 4.7의 `AHRS_OPTIONS` bit 4를 켜 재부팅 때 복원하도록 했다.
`AHRS_ORIGIN_ALT=0`은 상대 주행용 고도 기준이며 현장 해발고를 측량한 값이 아니다.
진북/GPS waypoint/절대고도 운용용 지리 정합으로 해석하지 않는다.
[공식 origin 저장/복원 설명](https://ardupilot.org/rover/docs/common-non-gps-navigation-landing-page.html).
다른 기체에서 origin을 수동 지정할 때는 다음을 참고한다.
Mission Planner의 EKF origin 설정 또는 아래 MAVROS 인터페이스로 운영자가 지정한다.
`LAT`, `LON`, `ALT_ELLIPSOID_M`에는 확인한 기준점 위도·경도·WGS84 타원체고를 넣는다.
MAVROS가 타원체고를 MAVLink의 해발고로 변환하므로 해발고를 그대로 넣지 않는다.

```bash
ros2 topic pub --once /mavros/global_position/set_gp_origin geographic_msgs/msg/GeoPointStamped \
  "{position: {latitude: ${LAT}, longitude: ${LON}, altitude: ${ALT_ELLIPSOID_M}}}"
ros2 topic echo /mavros/global_position/gp_origin --once --qos-durability transient_local
```

정지 시 카메라/FCU 속도가 모두 거의 0인지, 수동으로 천천히 전진·우회전할 때
두 추정의 속도 부호와 회전 방향이 일치하는지 확인한다. 모터 출력 시험과 분리한다.
`/external_navigation/status`와 FCU 로그 VISO/EKF를 함께 확인한다.
GPS를 비활성화했으므로 QGC의 GPS 위성 수/No Fix 자체를 성공 기준으로 삼지 않는다.
ROS `ready=true`만으로 외부 입력 정확도나 실차 주행 성공을 확정하지 않는다.

### 3. 검증 후 동일한 주행 launch 사용

validation을 정상 종료하고 차량이 disarmed인 상태에서 시작한다.

```bash
ros2 launch rover_navigation navigation.launch.py \
  mode:=drive map_dir:="$MAP_DIR" fcu_estimation:=externalnav \
  start_mavros:=false start_camera:=false start_camera_tf:=false rviz:=true
```

`drive`의 ExternalNav 전송은 자동으로 켜진다. 속도·조향 게인은 기존 그대로다.
준비가 확인된 뒤 운영자가 GUIDED/arm과 RViz goal을 진행한다. 처음에는 짧은 직선과 넓은 커브에서
목표/실제 속도, yaw-rate와 VO 유지 여부를 대조한다.
정지 융합 확인 후 launch 기본값도 externalnav로 바꿨다. 위 `fcu_estimation` 인자는 생략해도 된다.
두 개의 주행 launch를 따로 관리하거나 GPS 상태를 보고 입력을 자동 전환하지 않는다.

GPS 방식으로 복귀하려면 정지·disarm 후 **백업한 FCU 설정을 복원하고 재부팅**한 다음
`fcu_estimation:=gps`로 실행한다. launch 옵션만 바꿔도 FCU 설정이 복원되는 것은 아니다.

### 이번 기체의 백업과 롤백

백업 위치: `~/ros2_ws/backups/fcu/20261022_externalnav/`.
`original.params`는 전체 914개 QGC 형식 원본, `original.json`과 `original.sha256`은
기체 식별 정보 및 무결성 확인용이다. `journal.json`에는 **실제 변경한 값의 원래 값**을
쓰기 전에 저장한다. 롤백은 그 값만 복원하므로 다른 조향 튜닝을 덮어쓰지 않는다.
펌웨어, PWM 제한, 속도·조향 게인, 시동 검사 설정은 바꾸지 않았다.

차량을 정지·disarm/HOLD로 두고 navigation과 MAVROS를 정상 종료한다.
QGC가 같은 USB에 직접 접속했다면 연결을 해제한다. 이후 실행한다.

```bash
python3 ~/ros2_ws/src/tools/fcu_externalnav.py rollback \
  --directory ~/ros2_ws/backups/fcu/20261022_externalnav
python3 ~/ros2_ws/src/tools/fcu_externalnav.py reboot \
  --directory ~/ros2_ws/backups/fcu/20261022_externalnav
# USB가 다시 연결된 후 실행. 일시적인 Device busy이면 잠시 후 재시도한다.
python3 ~/ros2_ws/src/tools/fcu_externalnav.py verify-rollback \
  --directory ~/ros2_ws/backups/fcu/20261022_externalnav
```

`rollback`은 숨겨질 VISO 세부 값을 먼저 복원하고 `VISO_TYPE`을 마지막으로 되돌린다.
`verify-rollback`은 재부팅 뒤 다시 조회 가능한 원본 항목을 확인한다.
GPS 구성으로 돌아가면 GPS·나침반 문제 역시 원래대로 남으므로 복원 성공이 그 센서의
정상 상태를 보장하지는 않는다. 유지보수 도구는 다른 기체/펌웨어, armed 상태,
점유된 USB, 훼손된 백업에서는 쓰기를 거부하며 arm/주행 명령을 보내지 않는다.

## 구현 확인 자료

- [실제 FCU 버전의 ODOMETRY 처리](https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/GCS_MAVLink/GCS_Common.cpp): 지원 frame, pose/velocity 전달.
- [MAVROS ROS 2 router](https://github.com/mavlink/mavros/blob/2.12.0/mavros/src/lib/mavros_router.cpp): UAS별 raw MAVLink sink.
- [Rover EKF failsafe](https://ardupilot.org/rover/docs/common-ekf-inav-failsafe.html): 추정 이상 시 HOLD.
- [QGC parameter 파일 형식](https://github.com/mavlink/qgroundcontrol/blob/master/docs/en/qgc-dev-guide/file_formats/parameters.md): 기체·컴포넌트·이름·값·형식.

이 방법은 GPS/자기장 의존성을 제거하지만 영상 추적 한계까지 제거하지 않는다.
현장 프레임률·지연·부호와 FCU 융합을 먼저 확인하고, 그 결과가 부족하면 센서/연산 경로를 개선한다.
주행이 되게 만들려고 불량 오도메트리를 정상으로 표시하거나 실패 기준을 무조건 해제하지 않는다.

## 실패 후 전체 launch를 다시 켜지 않고 복구

지도 정합이 매 프레임 성공하지 않아도 주행한다. 골이 없을 때의 일시적인 ExternalNav 준비 지연은
영구 정지로 만들지 않는다. 주행 중 정상 측정/FCU 추정이 이어지면 평균 Hz 변화만으로
준비를 해제하지 않는다. 추적 상실을 포함한 유효 입력 2초 초과 단절·FCU 추정 이상 등
실제 실패는 정지·목표 취소 후 운영자 확인을 요구한다.
FCU 파라미터 조회 RPC가 잠깐 늦어진 것만으로 정상 주행을 중단하지 않는다. 마지막으로 확인한 설정을
유지하되 실제 파라미터 변경 이벤트/조회 불일치와 FCU 연결 단절은 별도로 반영한다.

시동 전의 일시적 영상/지연 문제는 정상 입력이 돌아오면 자동으로 재준비한다.
시동 중 발생한 실패, 오래된 FCU 상태에서 발생한 실패, 좌표 리셋은 자동 해제하지 않는다.
일반적인 주행 중 영상 추적 상실·입력 단절 뒤에는 차량을 멈추고 disarm한다. 정상 오도메트리가 다시
들어오면 아래 명령을 순서대로 실행한다. 각 서비스의 `success: true`를 확인한다.

```bash
ros2 service call /external_navigation/recover std_srvs/srv/Trigger '{}'
ros2 topic echo /external_navigation/ready
# true가 된 뒤 echo를 Ctrl+C로 종료하고 다음 명령을 실행한다.
ros2 service call /navigation/acknowledge_tracking_stop std_srvs/srv/Trigger '{}'
```

브리지 fault가 없고 FCU 준비 상태만 일시적으로 나빴다면 첫 서비스는 필요 없다.
`/navigation/ready` 회복 후 운영자가 GUIDED/arm과 **새 골**을 지정한다.
기존 골·속도 명령은 재사용하지 않는다. 지도 DB와 RTAB/Nav2 프로세스는 계속 유지된다.

좌표·시간 리셋이 검출됐거나 운용자가 명시적으로 emergency stop을 걸었다면 이 복구 명령으로
우회하지 않는다. 좌표 리셋은 지도상의 위치를 확인하고 브리지를 재시작해야 한다.
원인이 해결되지 않은 입력, 움직이는 차량, armed 상태에서는 브리지 복구를 거부한다.

## 검증 범위

FCU 설정의 재부팅 유지와 정지 상태 EKF 융합, 격리 ROS의 실제 MAVLink ODOMETRY 전송·
타임스탬프·추적 상실·복구를 확인했다. 카메라 기반 실차 주행 이력은 있지만 최신 융합 구성의
실외 목적지 도착, 속도·좌우 조향 추종, 통신 단절 시 제동 거리는 후속 현장 검증 대상이다.
센서 Hz나 FCU 수신 성공만으로 차량 제어가 검증됐다고 판단하지 않는다.

## RC 수신기 없는 운용 — 2026-10-27

현재 차량은 RC 수신기 없이 Jetson의 MAVLink 속도·각속도 명령으로 운용한다.
FCU의 `FS_THR_ENABLE=1`이 수신기 부재를 신호 단절로 판단해
`Arm: Radio failsafe on`으로 시동을 거부했으므로, disarmed 상태에서
**`FS_THR_ENABLE=0`만 적용하고 readback을 확인**했다.
`FS_ACTION=2`, `FS_GCS_ENABLE=0`, `RC_OPTIONS=32`, `FS_EKF_ACTION=1`은 유지했다.
전체 시동 검사 해제, 강제 시동, 자동 모드 전환은 수행하지 않는다.

Jetson 어댑터는 제어 입력이 0.5초 끊기면 0 명령을 보내며 이동 중에는 HOLD를 요청한다.
현재 ArduRover 4.7.1(`dbe79216`)의 속도·각속도 GUIDED 모드는 명령이 3초
갱신되지 않으면 정지 처리를 한다. 이 버전에는 `GUID_TIMEOUT` 파라미터가 없고
[펌웨어에 3000ms로 구현되어 있다](https://github.com/ArduPilot/ardupilot/blob/dbe79216/Rover/mode_guided.cpp#L71-L95).
GCS heartbeat 감시는 꺼진 상태이므로 QGC 접속 해제가 정지 조건은 아니다.
RC 수신기 미사용으로 원격 수동 조종에 의한 개입은 제공되지 않는다.
정지 처리의 실제 제동 거리나 통신 단절 실차 시험을 이번 파라미터 변경으로 검증한 것은 아니다.

변경 전후·복원 값은 Jetson의
`~/.local/share/campus-tools/maintenance/no-rc-20261027-125153.json`에 저장했다.
RC 수신기를 다시 사용하는 구성으로 복귀할 때만 차량 정지·시동 해제 후 복원한다:

```bash
ros2 param set /mavros/param FS_THR_ENABLE 1
ros2 param get /mavros/param FS_THR_ENABLE
```

매번 런치에서 설정을 덮어쓰지 않는다. 센서와 지도 설정에는 변경이 없다.
