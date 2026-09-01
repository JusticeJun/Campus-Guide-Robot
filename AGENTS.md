# AGENTS.md

이 저장소에서 작업하는 Codex/AI 개발 에이전트는 아래 원칙을 따른다.

## 1. 작업 시작과 범위

작업 시작 시 다음을 필요한 범위에서 확인한다.

1. 현재 branch와 `git status`
2. 작업과 관련된 package, node, module, symbol
3. `docs/architecture.md`의 관련 부분
4. 필요한 launch, config, parameter, ROS interface

항상 가장 좁은 범위에서 시작한다.

- repository 전체보다 관련 package/module/symbol을 먼저 확인한다.
- 원인 규명이나 영향 범위가 불명확할 때만 단계적으로 탐색 범위를 넓힌다.
- 이미 확인했고 변경되지 않은 파일이나 정보를 반복해서 읽지 않는다.
- 문서화된 구조를 매 작업마다 저장소 전체를 탐색해 다시 분석하지 않는다.
- 사용자의 기존 변경사항이나 다른 미완성 작업을 임의로 수정, 삭제, 되돌리지 않는다.
- 작업과 무관한 리팩터링이나 미래 기능을 위한 과도한 추상화를 하지 않는다.

`docs/architecture.md`를 전체 설계의 기준으로 사용한다.
문서와 실제 구현이 다르면 실제 상태를 확인하고 차이를 명확히 한다.

correctness와 safety는 context/token 절약보다 우선한다.


## 2. 프로젝트 설계 원칙

이 프로젝트는 Jetson과 Pixhawk/ArduRover 기반 Ackermann rover의
ROS 2 자율주행 시스템이다.

책임은 가능한 한 다음과 같이 분리한다.

- global route: GPS waypoint graph와 목적지 경로
- localization: GPS, IMU, SLAM, sensor fusion
- navigation: 경로 추종, 차량 제약, local navigation
- perception: 장애물 및 환경 인식
- hardware interface: MAVROS/Pixhawk/ArduRover와 actuator 경계

현재 구현되지 않은 목표 architecture를 이미 존재하는 기능처럼 가정하지 않는다.
임시 구현을 근거 없이 최종 architecture로 확장하지 않는다.

세부 node/topic 구조, 향후 Nav2 구성, TF/localization 설계 등은
`docs/architecture.md`를 기준으로 한다.

차량 치수, 센서 장착, Pixhawk 및 실제 하드웨어 특성은
가능하면 `docs/hardware.md`에 기록하고 AGENTS.md에 중복하지 않는다.


## 3. Git / GitHub

기본 흐름:

Implementation → 필요한 local validation → 사용자 실차 테스트 → 사용자 승인
→ Commit/Push/Pull Request → CI → CI 성공 → 사용자 최종 승인 → develop Merge

- `main`: stable/release
- `develop`: integration
- 기능 개발과 수정은 별도 branch에서 수행한다.
- `feat/`, `fix/`, `refactor/`, `chore/`, `docs/`, `test/` 등을 사용한다.
- 이미 같은 목적의 적절한 branch에 있다면 불필요하게 새 branch를 만들지 않는다.
- commit, Issue, PR은 영어로 작성한다.
- commit은 가능한 경우 Conventional Commits를 사용한다.
- 하나의 Issue/Branch/PR은 가능한 한 하나의 명확한 목적을 가진다.
- 사용자의 승인 없이 merge하지 않는다.
- destructive Git operation이나 force push를 임의로 수행하지 않는다.
- 실차 navigation 변경은 사용자가 실차 테스트를 완료하고 명시적으로 승인하기 전까지
  push, PR 생성, GitHub CI 실행 또는 develop merge를 하지 않는다.

CI 실패와 CI 자체가 실행되지 않은 경우를 구분한다.
CI를 다시 실행하기 위한 의미 없는 commit, 반복 push, PR 재생성을 하지 않는다.


## 4. ROS interface와 parameter

Node/package 책임을 불필요하게 침범하지 않는다.

Topic, service, action을 변경할 때 필요한 범위에서 다음을 확인한다.

- producer / consumer
- message type
- QoS
- frame과 좌표계
- launch/config
- 관련 test와 diagnostics

MAVROS command를 변경할 때는 특히 다음을 확인한다.

- coordinate frame
- type mask
- 단위
- active/ignored field
- Pixhawk/ArduRover에서의 실제 해석
- stop/watchdog 동작

같은 actuator command 경로에 여러 publisher가 동시에 개입하지 않도록 한다.

Parameter는 코드 기본값만 보고 실제 적용값을 단정하지 않는다.
launch, YAML/config, CLI override와 runtime 값을 필요한 범위에서 확인한다.

차량 관련 값의 단위를 명확히 유지한다.

- distance / radius: m
- speed: m/s
- yaw rate: rad/s
- heading/bearing: deg 또는 명시된 단위
- curvature: 1/m
- actuator output: PWM


## 5. Localization / Navigation / Vehicle Safety

GPS, compass, IMU, SLAM, EKF 등의 문제와
navigation/controller 문제를 먼저 구분한다.

TF/localization 변경 시 필요한 범위에서 다음을 확인한다.

- `map`, `odom`, `base_link`, sensor frame 관계
- ENU/NED convention
- heading convention
- degree/radian
- sensor mounting orientation/offset
- timestamp와 freshness

Ackermann rover가 제자리 회전할 수 있다고 가정하지 않는다.
물리적 최소 회전반경과 planning에 사용할 안전 회전반경을 구분한다.

planner의 경로, controller의 motion command,
MAVROS/Pixhawk command, 실제 actuator output을 서로 구분한다.

software가 계산한 speed/yaw-rate와 실제 PWM을 같은 값처럼 해석하지 않는다.

다음 safety behavior는 관련 변경에서 regression 여부를 확인한다.

- watchdog
- stale sensor/input 처리
- waypoint/route completion
- stop command
- command timeout
- invalid input
- shutdown 시 안전 정지

Pixhawk parameter나 firmware 변경은 ROS 코드 변경과 별도 영향 범위로 취급하며
명시적 요청 없이 변경하지 않는다.


## 6. Hardware와 실차 검증

실제 hardware가 필요한 검증과 software 검증을 분리한다.

가능한 경우 다음 순서에서 필요한 단계까지만 사용한다.

unit/deterministic test
→ ROS component/interface
→ launch/integration
→ recorded data 또는 simulation
→ hardware-in-the-loop
→ 실제 rover

모든 변경에서 모든 단계를 실행하지 않는다.

실차를 움직이는 테스트는 실행 전에 최소한 다음을 확인한다.

- mode / armed 조건
- 예상 motion과 actuator 방향
- 안전한 출력 범위
- stop/watchdog
- 중단 방법
- 관찰할 telemetry

software test 성공을 실제 rover 동작 검증이라고 표현하지 않는다.
실차에서 발견된 문제는 가능하면 deterministic regression으로 남긴다.


## 7. 검증 전략

개발 중에는 변경과 가장 가까운 targeted test부터 실행한다.

구현이 안정화되면 변경 영향 범위에 필요한 test/lint/build를 수행한다.

- 작은 변경마다 전체 workspace를 반복 build/test하지 않는다.
- 실패한 test/lint를 비활성화하거나 무시해서 통과시키지 않는다.
- 같은 원인과 입력의 실패를 결과 변화 없이 반복 실행하지 않는다.
- 테스트 개수보다 어떤 failure/regression을 검증했는지를 중요하게 본다.

실패 시 먼저 다음을 구분한다.

- implementation regression
- configuration/parameter
- dependency/environment
- sensor/hardware
- existing failure
- CI/infrastructure

검증하지 못한 단계는 통과했다고 가정하지 않는다.


## 8. Context / Token 사용량

Correctness와 safety를 유지하면서 불필요한 탐색과 출력을 최소화한다.

기본 원칙은:

좁은 범위 확인
→ 필요한 증거 확보
→ 부족하거나 모호할 때만 단계적으로 범위 확장

이다.

### Repository

- repository-wide search보다 관련 package/file/symbol 검색을 우선한다.
- `build/`, `install/`, `log/`, generated files는 기본 검색 대상에서 제외한다.
- 대형 BIN, rosbag, CSV, log는 작업에 실제로 필요할 때만 읽는다.
- 큰 파일은 전체 출력보다 symbol/line 위치를 먼저 찾고 필요한 주변만 읽는다.
- `git diff`는 먼저 `--stat` 또는 파일 목록을 보고 필요한 diff만 확인한다.
- Git history도 원인 규명에 필요한 commit 범위만 확인한다.
- 이미 확인했고 변경되지 않은 파일을 반복해서 읽지 않는다.

### Search / command output

검색 결과가 클 가능성이 있으면 파일/디렉터리 범위를 먼저 제한한다.

가능하면 다음과 같은 출력 제한을 사용한다.

- `rg --max-count`
- `rg --max-columns`
- 필요한 file glob 또는 directory
- line/symbol 범위 제한

특히 `/opt/ros`, generated MAVLink header, `build/`, `install/`에 대한
무제한 recursive search를 피한다.

예상보다 거대한 결과가 반환되면 같은 방식으로 반복하지 말고
즉시 더 좁은 검색 방법으로 전환한다.

### Web / external source

외부 source 조사도 symbol과 필요한 사실부터 좁게 확인한다.

- 전체 source보다 symbol/find/line-range 확인을 우선한다.
- GitHub source는 가능하면 version-tagged raw source를 사용한다.
- 코드 대신 거대한 HTML 페이지가 반환되면 그 방식의 탐색을 중단한다.
- tool output은 기본적으로 short/medium 범위를 사용한다.
- long output은 좁은 범위로 필요한 정보를 얻을 수 없을 때만 사용한다.
- 하나의 구현 사실은 신뢰할 수 있는 공식 source로 확인되면
  특별한 충돌이나 불확실성이 없는 한 같은 사실을 반복 검색하지 않는다.
- 공식 문서는 의미/사용법 확인에, source는 실제 구현 확인에 사용하고
  같은 내용을 여러 경로로 불필요하게 중복 검증하지 않는다.

좁은 탐색만으로 원인을 특정할 수 없거나
변경 영향 범위가 불명확하면 필요한 만큼 탐색 범위를 확장한다.

사용량 절약을 이유로 correctness, regression, safety 문제를
추측으로 처리하거나 미해결 상태로 넘기지 않는다.


## 9. 완료와 보고

다음 조건을 만족하면 작업을 완료한다.

- 요청한 범위가 구현 또는 조사되었다.
- 관련 correctness/regression을 확인했다.
- 필요한 영향 범위 검증이 완료되었다.
- hardware 검증 여부를 구분했다.
- 기존 사용자 변경을 보존했다.
- 알려진 미해결 사항을 명확히 했다.

완료 후 요청과 무관한 추가 탐색, 리팩터링, 테스트, 문서화를 하지 않는다.

완료 보고는 간결하게 다음 중 필요한 것만 포함한다.

- 무엇을 변경하거나 확인했는지
- 핵심 원인 또는 behavior 변화
- test / lint / build 결과
- 실제 hardware에서 확인할 사항
- 미검증/제한 사항
- branch / commit / Issue / PR 상태

수행하지 않은 작업을 수행한 것처럼 표현하지 않는다.
