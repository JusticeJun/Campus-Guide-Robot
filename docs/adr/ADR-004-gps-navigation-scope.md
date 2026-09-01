# ADR-004: GPS Navigation의 적용 범위와 정밀 Localization 방향

## Context / Problem

Standalone M8N GPS의 multi-meter uncertainty와 GPS-based Nav2의 실차 주행
성공을 함께 고려해, GPS navigation을 campus에서 어느 범위까지 사용할지
결정해야 했다.

## Decision

GPS는 clearance가 충분한 공간의 campus-scale/global route navigation에 계속
사용한다. 다만 GPS-only localization을 좁은 보행로, curb 인접 구간,
정밀 회전 구간까지 포함하는 최종 precise localization으로 간주하지
않는다.

장기적으로는 GPS global navigation을 유지하면서 정밀 3D map과
map/sensor-based localization을 결합하는 방향으로 발전시킨다. 이 정밀
localization architecture는 아직 구현과 실차 검증이 완료되지 않았다.

## Evidence

- Stationary M8N sampling에서 multi-meter scatter가 관찰되어 단발 GPS
  coordinate를 centimeter-level ground truth로 취급할 수 없음을 확인했다.
- Production 실차 시험에서 `W09 -> W07 -> W02` GPS-based Nav2
  multi-waypoint route를 완주했다.
- 넓고 거의 직선인 구간에서 periodic replanning과 closed-loop RPP
  correction으로 목적지에 도달했고, production bag에서 smoother/adapter/FCU가
  새로운 oscillation을 생성했다는 증거는 없었다.

## Consequences

- GPS navigation 성공과 GPS 정밀도 한계를 동시에 인정한다. GPS를
  폐기하거나 Nav2 실패로 기록하지 않는다.
- 현재 periodic replanning과 RPP correction을 이 시험만으로 버그나 제거해야
  할 oscillation으로 판단하지 않는다.
- 좁은 구간의 운용 확대는 map-based localization의 lateral accuracy와
  collision clearance가 실차에서 검증된 뒤에만 가능하다.
