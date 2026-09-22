# ADR-002: Nav2를 최종 Navigation Framework로 채택

## Context / Problem

Custom GPS navigator는 single target, multi-waypoint, graph shortest-path navigation까지
동작했다. 다음 단계에서는 Ackermann feasible path generation, continuous
path tracking, controller, costmap, sensor/obstacle, map/localization integration을
custom implementation에 계속 추가할지 결정해야 했다. Visual SLAM과 camera
기반 방법도 탐색했지만 production navigation architecture로 확정하지는
않았다.

## Decision

Campus graph는 global route를 계속 제공하고, 실제 path planning과 tracking은
Nav2에 위임한다.

```text
Campus Route Graph
  → Nav2 / Smac Hybrid-A*
  → Regulated Pure Pursuit
  → Velocity Smoother
  → Pixhawk Command Adapter
  → MAVROS → Pixhawk / ArduRover
```

Ackermann navigation constraint는 실차 회전반경 검증에 근거한 1.7 m로
정하고, Smac Hybrid-A* planner와 Pixhawk Command Adapter가 같은 제약을
사용한다.

## Evidence

- Fixed-circle 실차 시험에서 steady-state physical turning radius가 약
  1.66--1.67 m임을 확인해 navigation에는 보수적으로 1.7 m를 적용했다.
- 현재 production launch에서 graph route가 Nav2 goal로 변환되고 planner,
  controller, smoother, command adapter를 통해 FCU로 전달된다.

## Consequences

- Custom navigator의 실차 성공은 유효한 baseline으로 남지만 production
  navigation framework는 Nav2다.
- Global route, feasible planning, path tracking, command adaptation의 책임이 분리된다.
- Planner와 command boundary의 물리 제약이 일치한다.
- 현재 costmap에 obstacle source가 없으므로 sensor-based obstacle avoidance와
  map-based localization을 구현 완료된 기능으로 간주하지 않는다.
