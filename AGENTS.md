# AGENTS.md

## 1. 목적

이 파일은 이 프로젝트에서 작업하는 Codex 및 기타 코딩 에이전트가 따라야 할 공통 개발 규칙을 정의한다.

프로젝트는 여러 개발자 또는 여러 Codex 세션이 서로 다른 기능을 병렬로 개발할 가능성이 있으므로, 기존 구조와 다른 작업자의 변경 사항을 최대한 존중하고 충돌 가능성을 최소화해야 한다.

단순히 기능이 동작하도록 코드를 작성하는 것뿐만 아니라, 실제 우수한 개발자가 장기간 유지보수하는 프로젝트처럼 코드와 Git/GitHub 이력을 체계적으로 관리한다.

---

## 2. 작업 시작 전 원칙

코드를 수정하기 전에 반드시 현재 프로젝트를 먼저 이해한다.

최소한 다음을 확인한다.

- 현재 Git branch
- `git status`
- 기존 modified / staged / untracked 파일
- 최근 commit history
- 프로젝트 디렉터리 구조
- 현재 작업과 관련된 package/module/node
- 기존 코드의 책임과 데이터 흐름
- 다른 기능과의 의존 관계

기존 구조를 충분히 확인하지 않은 상태에서 새로운 구조를 임의로 도입하지 않는다.

사용자의 기존 변경 사항이나 다른 개발자가 작업 중인 코드를 임의로 삭제하거나 되돌리지 않는다.

---

## 3. Git Flow

이 프로젝트는 전통적인 Git Flow 방식을 기본으로 사용한다.

### `main`

`main`은 안정적이고 릴리스 가능한 상태의 코드를 유지한다.

일반적인 기능 개발이나 버그 수정을 `main`에서 직접 진행하지 않는다.

### `develop`

`develop`은 개발 결과가 통합되는 기본 개발 브랜치다.

새로운 기능이나 수정 작업은 원칙적으로 최신 `develop`을 기준으로 별도의 branch를 생성하여 진행한다.

### Feature branch

새로운 기능은 다음과 같은 branch에서 개발한다.

```text
feature/<short-descriptive-name>
```

예:

```text
feature/obstacle-avoidance
feature/web-interface
feature/gps-navigation
```

### Fix branch

버그 수정은 필요에 따라 다음 형식을 사용한다.

```text
fix/<short-descriptive-name>
```

예:

```text
fix/steering-control
fix/heading-calculation
fix/waypoint-transition
```

### Release / Hotfix

프로젝트 상황에 따라 필요한 경우 다음 branch를 사용할 수 있다.

```text
release/<version>
hotfix/<short-descriptive-name>
```

Git Flow 형식을 흉내 내기 위해 의미 없는 branch를 생성하지 않는다.

---

## 4. 사용자 승인 전 `develop` 병합 금지

Codex가 작성한 코드가 build/test를 통과했다는 이유만으로 작업을 완료된 것으로 간주하지 않는다.

이 프로젝트에는 실제 차량, 센서, Pixhawk, 서보 등 실제 하드웨어에서 확인해야 하는 동작이 존재할 수 있다.

따라서 기능 구현 후:

1. 작업 branch에서 코드를 완성한다.
2. 가능한 build/test/static check를 수행한다.
3. 필요한 경우 remote에 branch를 push한다.
4. 필요한 경우 Pull Request를 생성한다.
5. 사용자에게 변경 내용과 테스트 방법을 설명한다.
6. 사용자가 실제 환경에서 동작을 확인할 수 있도록 한다.
7. **사용자가 명시적으로 테스트 성공 및 병합을 승인한 이후에만 `develop`에 병합한다.**

사용자의 승인 없이 작업 branch 또는 PR을 임의로 `develop`에 merge하지 않는다.

특히 실제 하드웨어 동작이 관련된 기능은 사용자의 육안 확인 또는 실제 주행 테스트를 소프트웨어 테스트와 구분한다.

---

## 5. GitHub 관리

필요하다면 Codex는 GitHub를 적극적으로 활용한다.

작업 성격에 따라 적절한 경우 다음을 수행할 수 있다.

- branch push
- Pull Request 생성 및 업데이트
- Issue 생성 및 업데이트
- 관련 Issue와 PR 연결
- commit 정리
- 작업 진행 상황 기록
- 테스트 결과 기록
- 리뷰를 위한 변경 사항 정리

단순히 형식적인 기록을 만들기 위해 불필요한 Issue나 PR을 남발하지 않는다.

GitHub 기록은 실제 프로젝트의 개발 과정이 명확하게 드러나도록 관리한다.

저장소를 봤을 때 숙련된 개발자가 일관된 규칙 아래 프로젝트를 관리하고 있다는 인상을 줄 수 있도록 branch, commit, Issue, PR의 목적과 관계를 명확하게 유지한다.

---

## 6. Git/GitHub에서 사용하는 언어

이 `AGENTS.md`의 설명은 사용자가 직접 확인할 수 있도록 한글로 작성한다.

반면 Git/GitHub에 실제로 기록되는 개발 관련 텍스트는 **영어를 사용한다.**

다음 항목은 영어로 작성한다.

- Branch 이름
- Commit message
- Pull Request 제목
- Pull Request 설명
- Issue 제목
- Issue 설명
- Merge 관련 메시지
- Release 관련 텍스트
- GitHub상 개발 기록

Commit message는 가능한 경우 Conventional Commits 형식을 사용한다.

예:

```text
feat: add obstacle avoidance pipeline
fix: correct steering command scaling
refactor: isolate navigation control logic
test: add heading normalization tests
docs: update navigation documentation
chore: update development configuration
```

다음과 같이 의미가 불명확한 commit message는 피한다.

```text
update
fix
changes
final
working
test
```

Git history만 보더라도 프로젝트가 어떤 과정으로 개발되었는지 이해할 수 있도록 한다.

---

## 7. Commit 원칙

Commit은 논리적인 작업 단위로 나눈다.

각 commit은 가능한 한 하나의 명확한 목적을 가진다.

서로 관계없는 변경 사항을 하나의 commit에 포함하지 않는다.

반대로 의미 없는 작은 수정마다 지나치게 많은 commit을 생성하지 않는다.

Commit 전에 반드시 변경 내용을 확인하고 현재 작업과 관계없는 파일이 포함되지 않았는지 확인한다.

가능한 경우 commit 전에 관련 build/test를 수행한다.

---

## 8. Pull Request 원칙

Pull Request는 가능한 한 하나의 기능 또는 하나의 수정에 집중한다.

PR에는 필요에 따라 다음 내용을 포함한다.

- 변경 목적
- 주요 변경 사항
- 중요한 구현 결정
- 테스트 방법
- 테스트 결과
- 실제 하드웨어에서 추가로 확인해야 하는 사항
- 관련 Issue

PR은 사용자가 최종 동작을 확인하기 위한 리뷰 단위로도 활용한다.

PR이 생성되어 있다는 사실 자체를 `develop` 병합 승인으로 간주하지 않는다.

사용자가 테스트를 완료하고 명시적으로 승인하기 전까지 PR을 merge하지 않는다.

---

## 9. Issue 원칙

독립적인 버그, 향후 기능, 기술 부채 또는 별도의 추적이 필요한 작업은 필요에 따라 GitHub Issue로 관리할 수 있다.

Issue를 생성할 때는:

- 문제 또는 목표를 명확하게 설명한다.
- 현재 증상과 기대 동작을 구분한다.
- 필요한 경우 관련 branch/PR과 연결한다.
- 해결된 경우 실제 해결 내용과 일치하도록 관리한다.

단순한 몇 줄 수정처럼 별도의 추적 가치가 없는 작업까지 무조건 Issue로 만들 필요는 없다.

---

## 10. 병렬 개발 및 충돌 방지

다른 개발자 또는 다른 Codex가 동시에 다른 기능을 개발하고 있을 수 있다고 가정한다.

따라서:

- 현재 작업에 필요한 범위만 수정한다.
- 관계없는 package/module/file을 수정하지 않는다.
- 기존 public interface/topic/message/configuration을 불필요하게 변경하지 않는다.
- 공용 코드 변경이 필요한 경우 영향 범위를 먼저 확인한다.
- 다른 기능의 코드를 임의로 리팩터링하지 않는다.
- 파일 구조를 개인적인 선호만으로 재구성하지 않는다.
- 이미 존재하는 기능을 별도의 방식으로 중복 구현하지 않는다.
- 현재 작업 branch의 목적과 관계없는 변경을 포함하지 않는다.

충돌 가능성이 높은 공용 구조를 수정해야 한다면 최소 변경을 우선한다.

---

## 11. 프로젝트 구조 및 설계 문서 유지

이 `AGENTS.md`에는 Git/GitHub 규칙뿐만 아니라 새로운 개발자 또는 새로운 Codex가 프로젝트 방향을 잘못 이해하지 않도록 현재 프로젝트의 핵심 구조와 설계를 간략하게 기록할 수 있다.

프로젝트 구조나 주요 설계가 충분히 파악된 경우 이 파일의 아래 `프로젝트 구조 및 현재 설계` 영역을 최신 상태로 유지한다.

단, 추측해서 작성하지 않는다.

실제 워크스페이스의 코드, package 구조, 실행 흐름을 확인한 내용만 기록한다.

새로운 기능을 추가할 때마다 세부 구현 내용을 모두 기록할 필요는 없다.

다음 개발자가 프로젝트를 처음 보더라도 다음 내용을 빠르게 이해할 수 있는 수준이면 충분하다.

- 프로젝트의 전체 목적
- 주요 package/module/node의 역할
- 핵심 데이터 흐름
- 주요 외부 시스템과의 연결 관계
- 이미 구현된 핵심 기능
- 현재 유지해야 하는 중요한 설계 원칙
- 다른 기능 개발 시 충돌 가능성이 높은 공용 영역

프로젝트 구조나 핵심 아키텍처가 실제로 변경되었다면 필요에 따라 이 설명도 업데이트한다.

---

## 12. 기존 작업 보호

사용자의 작업이나 다른 개발자의 작업을 임의로 삭제하지 않는다.

명시적인 허가 없이 다음과 같은 destructive Git 명령을 사용하지 않는다.

```bash
git reset --hard
git clean -fd
git checkout -- .
git restore .
git push --force
git push --force-with-lease
```

기존 modified, staged, untracked 파일은 작업 시작 전에 반드시 확인한다.

현재 작업과 관계없는 변경 사항은 자신의 commit에 포함하지 않는다.

---

## 13. 코드 수정 원칙

현재 요청을 해결하는 데 필요한 최소 범위의 변경을 우선한다.

특별한 이유 없이 다음 작업을 하지 않는다.

- 대규모 리팩터링
- 프로젝트 디렉터리 재구성
- 정상 동작하는 기능 재작성
- 관계없는 파일 이름 변경
- 관계없는 dependency 업데이트
- 개인적인 취향에 따른 대량 포맷 변경

문제를 수정할 때는 가능한 한 증상이 아니라 root cause를 해결한다.

큰 구조 변경이 필요하다면 기존 구조와 영향 범위를 충분히 분석한 후 진행한다.

---

## 14. 테스트 및 검증

작업 완료 전 가능한 범위에서 다음을 수행한다.

- 변경 사항 검토
- 관련 package/component build
- 관련 automated test
- lint/static analysis
- regression 확인

소프트웨어 테스트와 실제 하드웨어 테스트를 명확하게 구분한다.

실제 장비에서 확인하지 않은 기능을 실제 환경에서도 정상 동작한다고 단정하지 않는다.

하드웨어 확인이 필요하면 사용자에게 테스트 방법과 확인해야 할 결과를 명확하게 설명한다.

---

## 15. 작업 결과 보고

작업 완료 후 사용자에게 최소한 다음 내용을 보고한다.

- 작업한 branch
- 문제의 원인 또는 구현 목표
- 변경한 주요 파일
- 구현 또는 수정 내용
- 생성한 commit
- 생성하거나 업데이트한 Issue/PR
- 수행한 테스트
- 테스트 결과
- 사용자가 실제 환경에서 확인해야 할 사항

사용자가 변경 내용을 이해하고 직접 검증할 수 있도록 설명한다.

---

# 프로젝트 구조 및 현재 설계

## 전체 목적과 워크스페이스 구성

이 저장소는 NVIDIA Jetson에서 ROS 2를 실행하고 MAVROS를 통해 Pixhawk 기반 차량을 제어하는 캠퍼스 안내 로봇을 개발한다. 저장소 자체는 ROS 2 워크스페이스의 `src`이며, 현재 소스 패키지는 Python 기반 `ament_python` 패키지인 `gps_navigator` 하나다. 워크스페이스 루트의 `build`, `install`, `log`는 `colcon` 산출물이며 소스 구조로 취급하지 않는다.

`gps_navigator/config/waypoints.yaml`은 캠퍼스 지점의 ID, 표시 이름, GPS 좌표와 인접 관계를 정의하는 공유 graph 데이터다. `gps_navigator/launch/gps_navigation.launch.py`는 아래 세 node를 조합하며, 실제 차량 명령을 내리는 waypoint follower는 launch argument로 명시적으로 활성화할 때만 실행한다.

## 주요 node와 책임

- `path_planner`: MAVROS GPS 위치에서 가장 가까운 graph waypoint를 출발점으로 정하고, 요청된 목적지까지 Dijkstra 최단경로를 계산해 route를 발행한다. 목적지는 waypoint ID 또는 고유한 표시 이름으로 지정한다.
- `route_manager`: 계획된 route와 현재 GPS를 받아 active waypoint를 관리한다. 도달 반경 안에 들어오면 다음 waypoint로 진행하고, 다음 목표 좌표·route 상태·완료 상태와 heartbeat를 발행한다.
- `waypoint_follower`: MAVROS의 GPS와 compass heading, route manager의 active waypoint·완료 상태·heartbeat를 결합한다. 목표 bearing과 signed heading error를 계산해 전진 속도와 yaw-rate setpoint를 만들고 Pixhawk 방향으로 보낸다. 입력이 없거나 오래되었거나 route가 끝난 경우 정지 명령을 보낸다.
- `geo_utils`: 좌표 유효성 검사와 haversine 거리 계산을 planner와 route manager가 공유한다.

## 핵심 데이터 흐름과 외부 연결

기본 위치 입력은 Pixhawk/GPS가 MAVROS를 거쳐 제공하는 `/mavros/global_position/global`이고, heading 입력은 `/mavros/global_position/compass_hdg`다. 두 값과 graph의 좌표는 모두 북쪽을 0도로 하고 시계 방향으로 증가하는 geographic bearing 체계에서 결합한다.

```text
MAVROS GPS + 목적지
  → path_planner → /gps_navigation/route
  → route_manager → /gps_navigation/next_waypoint, route 상태/완료
  → waypoint_follower + MAVROS compass heading
  → BODY_NED 전진 속도 + yaw rate
  → /mavros/setpoint_raw/local → MAVROS → Pixhawk/ArduRover → 조향·구동 출력
```

route와 현재/완료 상태에는 late subscriber도 최신 값을 받을 수 있도록 reliable + transient-local QoS를 사용한다. 센서 입력은 MAVROS 특성에 맞춰 best-effort QoS를 사용한다. route manager heartbeat와 follower watchdog은 경로 관리 node 중단 시 오래된 목표로 계속 주행하지 않도록 하는 안전 경계다.

## 현재 구현 범위와 설계 방향

현재 구현 범위는 YAML graph 로딩·검증, 현재 위치 기반 시작점 선택, 최단경로 계산, route 진행 및 waypoint 도달 판정, 구간별 정지/재출발, GPS bearing 기반 조향 명령과 입력 watchdog이다. 장애물 회피, 동적 graph 갱신, Pixhawk mode 전환·arming, 저수준 servo/PWM 설정은 이 패키지의 현재 책임이 아니다.

계획(`path_planner`), 진행 상태(`route_manager`), 차량 명령(`waypoint_follower`)의 책임을 분리한다. 상위 node는 Pixhawk servo channel이나 PWM을 직접 구동하지 않고 MAVROS/MAVLink setpoint를 사용하며, 실제 조향 mixer·servo channel·PWM 범위는 Pixhawk/ArduRover 설정의 책임으로 유지한다. 기존 topic과 route message 형식은 node 사이의 공용 interface이므로 기능 추가 시 불필요하게 변경하지 않는다.

## 공용·충돌 주의 영역

- `config/waypoints.yaml`: planner와 route manager가 함께 사용하는 graph 원본이다. 좌표나 연결 변경은 경로 전체에 영향을 주므로 중복 waypoint 추가와 병렬 수정에 주의한다.
- `/gps_navigation/*` topic, JSON route/status 형식, transient-local QoS: 세 node 사이의 공용 계약이다.
- `geo_utils.py`: 위치 유효성 및 거리 의미를 공유하므로 호출자 전체의 영향을 확인한 뒤 변경한다.
- `launch/gps_navigation.launch.py`, `setup.py`, `package.xml`: 실행 node, 설치 데이터, dependency를 묶는 package 공용 진입점이다.
- `waypoint_follower`의 MAVROS 입력·출력 topic과 BODY_NED command frame: 실제 차량 연결 계약이다. 속도·조향 변경과 Pixhawk 설정 변경을 동시에 임의로 수행하지 말고 실제 차량 검증을 거친다.
