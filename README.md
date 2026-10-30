# Campus Guide Robot

ROS 2 Humble, NVIDIA Jetson Orin Nano, RealSense D435if, Pixhawk 6C/ArduRover로
캠퍼스 Ackermann 로버의 지도 생성과 자율주행을 구현한다.

현재 공개 범위는 **RGB-D 카메라·카메라 IMU 기반 RTAB-Map 매핑/위치추정과 Nav2 주행**이다.
기존 GPS 경로 그래프 방식도 별도 실행 경로로 유지한다. 라이다 실험 코드는 포함하지 않는다.

## 구현 범위

- 카메라/IMU 수집, RGB-D 오도메트리, RTAB-Map 새 지도·이어 촬영
- 지도별 DB·카메라 설정 보관과 원본을 보존하는 localization DB 캐시
- RViz 목표점 → Smac Hybrid-A* → RPP → 속도 평활화 → 보호 노드 → MAVROS/Pixhawk
- 위치추정·TF·경로·명령 유효성 검사와 추적 손실 시 정지/HOLD 요청
- SSH 연결이 끊겨도 유지되는 tmux 실행, 크기가 제한된 주행 기록
- 차량 회전반경·속도·조향 응답 및 GPS 샘플링 도구와 회귀 테스트

**현재 costmap은 정적 지도 기반이다. 새로 나타난 장애물의 감지·회피는 구현 범위에 포함하지 않는다.**
카메라 기반 지도 주행은 실차 시험을 진행했고 운영자가 주행 성공을 확인했다.
이것이 모든 조명·장소·속도에서의 위치추정이나 안전한 무인 운행을 보장하지는 않는다.
특히 FCU가 쓰는 GPS/나침반의 간섭 문제는 별도의 하드웨어 점검 대상이다.

## 시작하기

저장소는 ROS 작업 공간의 `src`에 둔다. 지도 DB와 실행 산출물은 Git에 포함하지 않는다.

```bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
git clone https://github.com/JusticeJun/Campus-Guide-Robot.git src
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-select localization_recovery_bt rover_navigation
source install/setup.bash
```

ROS 2 Humble/Ubuntu 22.04 환경을 사용한다. 실차 검증 환경은 Nav2 1.1.20,
RTAB-Map 0.23.7, RealSense ROS 4.58.4/librealsense 2.58.4다.
`rosdep`은 의존 패키지를 설치하지만 이 버전 조합이나 Jetson 카메라 IMU 커널 지원까지 고정하지 않는다.
MAVROS 지리 데이터가 없다면 설치 패키지의 `install_geographiclib_datasets.sh`도 실행해야 한다.
tmux를 사용하려면 `sudo apt install tmux`가 필요하다.

센서 준비부터 지도 저장, validation/drive 실행까지는 [실행 안내](docs/navigation.md)를 따른다.
지도 파일은 별도로 준비해야 하므로 복제 직후 실차 주행이 가능한 배포 이미지는 아니다.

## 구조와 문서

| 경로 | 역할 |
|---|---|
| `rover_navigation/` | ROS 노드, launch, 설정, Python 테스트 |
| `localization_recovery_bt/` | Nav2 위치추정 대기 BT 플러그인과 C++ 테스트 |
| `tools/ros_session*.sh` | 연결 끊김에 대비한 실행·로그 접속 도구 |
| [실행 안내](docs/navigation.md) | 지도 생성·저장·주행·복구 명령 |
| [구조](docs/architecture.md) | 토픽, TF 소유권, 명령과 정지 경로 |
| [하드웨어](docs/hardware.md) | 치수, 센서 장착, FCU 검증값과 한계 |
| [ADR](docs/adr/README.md) | 설계 결정과 변경 이유 |

## 검증

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 colcon test --packages-select rover_navigation localization_recovery_bt \
  --event-handlers console_direct+
colcon test-result --verbose
```

`PYTHONNOUSERSITE=1`은 사용자 홈에 설치된 무관한 pytest 플러그인이 ROS 시스템 테스트 환경을
덮어쓰지 않도록 한다. 기능 테스트는 센서·모터 없이 실행하며, 실차 검증과 구분한다.
기존 저작권 헤더 검사는 프로젝트 소스의 라이선스 정리가 완료되지 않아 건너뛰는 상태다.
