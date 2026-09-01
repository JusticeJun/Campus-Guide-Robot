# ADR-001: GPS 기반 Campus Route Navigation Architecture

## Context / Problem

프로젝트는 Jetson의 ROS 2 node가 MAVROS를 통해 Pixhawk/ArduRover 기반
Ackermann rover를 제어하는 outdoor navigation으로 시작했다. 단일 GPS
목적지를 넘어 campus 내 여러 지점을 순서와 연결 관계에 따라 주행할
전역 route 표현이 필요했다.

## Decision

Campus-scale global route는 GPS waypoint와 edge connectivity를 가진 graph로
표현하고, Dijkstra shortest path로 목적지 route를 결정한다. 현재 GPS를
가장 가까운 graph edge에 projection하여 최초 graph node 하나를 선택하고,
그 후에는 graph route를 그대로 사용한다. 이는 initial graph-entry
policy이지 global route shortcut mechanism이 아니다.

Vehicle command boundary는 ROS 2 → MAVROS → Pixhawk/ArduRover로 유지하며,
navigation software가 servo PWM을 직접 제어하지 않는다.

## Evidence

- 초기 custom navigator는 current GPS와 target GPS로 bearing을 구하고 compass
  heading error를 yaw command로 변환해 단일 목적지로 주행했다.
- 이 구조는 multi-waypoint 진행과 campus waypoint graph/Dijkstra route
  navigation으로 확장되어 실제 GPS navigation baseline으로 동작했다.
- Nearest waypoint start는 edge 중간 출발 시 목적지 반대 방향 node를 먼저
  선택할 수 있어 edge-based initial entry로 보완했다.

## Consequences

- GPS waypoint graph는 navigation framework와 독립된 campus global-route source로
  남는다.
- Graph은 허용된 연결 topology를 표현하지만, 차량이 추종할 연속
  feasible path와 장애물을 직접 해결하지는 않는다.
- GPS 좌표 정확도와 세부 motion planning은 별도 계층의 책임으로 분리한다.
