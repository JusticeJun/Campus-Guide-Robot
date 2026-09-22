# ADR-003: Navigation Planning과 Vehicle Control의 책임 분리

## Context / Problem

Nav2 초기 실차 시험의 steering undertracking을 해결할 때 Nav2 angular
command를 인위적으로 증폭하거나 minimum turning radius 제약을 약화하면
navigation이 생성한 physically feasible command의 의미가 깨진다. 문제가
navigation, command adaptation, FCU tracking 중 어느 계층에서 발생했는지
분리할 필요가 있었다.

## Decision

Navigation layer는 차량 제약 내의 path, velocity, yaw-rate를 생성하고,
Pixhawk/ArduRover vehicle-control layer는 그 command를 steering/throttle로 추종한다.
FCU undertracking은 Nav2 command scaling이 아니라 FCU steering-rate controller에서
해결한다.

## Evidence

- Nav2를 제거한 fixed-circle test로 FCU tracking을 독립적으로 검증했다.
- 0.4 m/s의 physically valid target yaw-rate에 대해 steady-state tracking이 약
  99%인 validated FCU baseline을 확립했다.
- Production status monitor와 rosbag은 navigation → smoother → adapter → FCU
  command chain을 계층별로 비교할 수 있게 한다.

## Consequences

- Navigation tuning이 low-level steering tracking 오차를 숨기지 않는다.
- Steering 이상은 planned path, controller output, smoothed command, FCU target,
  actual response를 분리해 진단한다.
- FCU parameter는 ROS launch가 자동 설정하지 않으며 hardware operation 전에
  validated baseline의 실제 적용 상태를 확인한다.
