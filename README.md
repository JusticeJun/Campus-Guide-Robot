# Campus Guide Robot

ROS 2 Humble, NVIDIA Jetson Orin Nano, RealSense D435if, Pixhawk 6C/ArduRover로
캠퍼스 Ackermann 로버의 지도 생성과 자율주행을 구현한다.

현재 소스는 **카메라·라이다 기반 RTAB-Map/Nav2 주행과 웹 기반 임무 수행 계층**으로 구성된다.
GPS 없이 카메라/융합 오도메트리를 Pixhawk ExternalNav에 전달하며, 기존 GPS 경로 그래프 코드는 별도 실행 경로로 남아 있다.

## 구현 범위

- RGB-D/IMU 수집, 카메라·라이다 오도메트리 융합, RTAB-Map 새 지도·이어 촬영
- 지도별 DB·카메라 설정 보관과 원본을 보존하는 localization DB 캐시
- RViz/웹 목표점 → Smac Hybrid-A* → Ackermann 전후진 RPP → 충돌 검사·보호 노드 → MAVROS/Pixhawk
- 라이다 코스트맵 갱신, 장애물 감속·정지·재계획
- 지도별 장소 등록, 웹 임무 시작·취소, 도착 후 촬영·외부 VLM 분석
- 부팅 웹 서비스와 사용자가 요청하는 센서·지도 준비/GUIDED·시동 절차
- 위치추정·TF·경로·명령 유효성 검사와 추적 손실 시 정지/HOLD 요청
- SSH 연결이 끊겨도 유지되는 tmux 실행, 크기가 제한된 주행 기록
- 차량 회전반경·속도·조향 응답 및 GPS 샘플링 도구와 회귀 테스트

카메라 기반 map1 주행과 라이다 회피는 실차 시험 이력이 있다.
후진·새 관찰 자세 도착 판정·웹 시작 절차는 소프트웨어/격리 ROS 검증과 실차 검증을 구분한다.
외부 API 키와 실제 관찰 목적지는 장비별로 설정해야 한다. 모든 환경의 무인 운행을 보장하는 배포 이미지는 아니다.

## 개발 상태

최신 통합 소스는 `develop`에서 관리한다. `main`은 과거 안정 기준선이며 최신 기능 배포판을 뜻하지 않는다.
기능 구현·소프트웨어 검증과 실차 운행 검증을 구분하며, 현장 확인이 남은 항목은 GitHub 이슈로 추적한다.

- 기존 카메라 지도 `map1`의 `local_odometry:=auto`는 카메라 추정을 사용한다. 라이다 회피는 별도 선택이다.
- 카메라·라이다로 만든 지도는 `auto`에서 융합을 선택한다. 기존 지도에서 융합을 시험하려면 `local_odometry:=fused`를 지정한다.
- 카메라 상실 시 라이다가 유효한 동안 로컬 추정은 이어갈 수 있지만, 목적지 도착이나 라이다 단독 전역 정합을 보장하지 않는다.
- 지역 노드 메모리 관리는 실험 옵션이다. 기본 `full`과 지연 특징점 로딩을 유지하며 10~20GB 실외 지도 운행은 미검증이다.

## 시작하기

저장소는 ROS 작업 공간의 `src`에 둔다. 지도 DB와 실행 산출물은 Git에 포함하지 않는다.

```bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
git clone https://github.com/JusticeJun/Campus-Guide-Robot.git src
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-select localization_recovery_bt ackermann_rpp rover_navigation
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
| `apps/campus_mission/` | 웹·임무 앱, 프런트엔드, 배포 설정, 테스트 |
| `ackermann_rpp/` | 전후진·곡률 제한을 적용하는 Nav2 controller |
| `localization_recovery_bt/` | Nav2 위치추정 대기 BT 플러그인과 C++ 테스트 |
| `tools/ros_session*.sh` | 연결 끊김에 대비한 실행·로그 접속 도구 |
| [웹 임무·자동 실행](docs/campus_missions.md) | 웹 접속·시동 준비·지도별 장소·API 설정 |
| [실행 안내](docs/navigation.md) | 지도 생성·저장·주행·복구 명령 |
| [구조](docs/architecture.md) | 토픽, TF 소유권, 명령과 정지 경로 |
| [하드웨어](docs/hardware.md) | 치수, 센서 장착, FCU 검증값과 한계 |
| [ADR](docs/adr/README.md) | 설계 결정과 변경 이유 |

소스와 실제 운영 데이터는 다음처럼 분리한다. **Git 저장소는 `src/`뿐이다.**

```text
~/ros2_ws/                       SSD의 ROS 작업 공간
├── src/                        Git: 코드·공개 설정·문서
│   ├── apps/campus_mission/     웹·임무 서비스
│   ├── rover_navigation/        주행·매핑·센서 통합
│   ├── ackermann_rpp/          차량 경로 추종
│   ├── localization_recovery_bt/
│   ├── docs/                   구조·운영 안내·ADR
│   └── tools/                  공통 실행·정비 진입점
├── maps/                       지도 DB·이미지·장소 좌표
├── mission_data/               임무 기록·사진
├── build/  install/  log/       ROS 빌드·실행 산출물
├── .cache/                     재사용 localization DB·ExternalNav 상태
└── backups/fcu/                기체 설정 복구에 필요한 최소 기록
~/.config/campus-rover/          비밀키·장비별 서비스 설정
~/.config/systemd/user/         설치한 사용자 서비스
```

`rover_navigation`은 기존 `gps_navigator`의 이름을 현재 역할에 맞게 변경한 패키지다.
카메라·라이다·매핑·Nav2·Pixhawk 연결을 포함하며, 이름 변경으로 속도·TF·주행 알고리즘은 바꾸지 않는다.
기존 명령의 패키지 이름만 `ros2 launch rover_navigation ...`으로 바꾼다.
`ackermann_rpp`와 `localization_recovery_bt`는 Nav2 플러그인으로 별도 유지한다.

홈 전체를 SSD에 옮길 때도 사용자 경로는 `/home/ykk/...`를 유지한다.
[저장장치 이전 절차](docs/navigation.md#홈-전체를-ssd로-옮기기)를 참고한다.

`instructions.md`는 이 장비의 개인 실행 메모로 Git에서 제외한다. 타인에게 필요한 절차는 `docs/`에 둔다.
웹 `dist/`와 dependency lockfile은 배포용으로 포함하고 `node_modules`, 키, 지도 DB, 사진, 로그는 올리지 않는다.

## 검증

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 colcon test --packages-select rover_navigation localization_recovery_bt ackermann_rpp \
  --event-handlers console_direct+
colcon test-result --verbose
```

`PYTHONNOUSERSITE=1`은 사용자 홈에 설치된 무관한 pytest 플러그인이 ROS 시스템 테스트 환경을
덮어쓰지 않도록 한다. 기능 테스트는 센서·모터 없이 실행하며, 실차 검증과 구분한다.
기존 저작권 헤더 검사는 프로젝트 소스의 라이선스 정리가 완료되지 않아 건너뛰는 상태다.
