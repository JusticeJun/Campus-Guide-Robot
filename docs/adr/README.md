# Architecture Decision Records

이 디렉터리는 실험 일지가 아니라, 실차와 구현으로 확인된 근거에 따라
선택한 장기적 architecture decision을 기록한다.

## Architecture Evolution

```text
GPS + Compass Custom Navigation
        ↓
Campus GPS Waypoint / Route Graph
        ↓
Nav2 Adoption
        ↓
Ackermann-aware Planning + FCU Command Tracking
        ↓
GPS-based Nav2 Real-world Validation
        ↓
GPS Global Navigation + Map-based Precise Localization Direction
```

## ADRs

1. [ADR-001: GPS 기반 Campus Route Navigation Architecture](ADR-001-gps-campus-route-navigation.md)
2. [ADR-002: Nav2를 최종 Navigation Framework로 채택](ADR-002-adopt-nav2.md)
3. [ADR-003: Navigation Planning과 Vehicle Control의 책임 분리](ADR-003-separate-navigation-and-vehicle-control.md)
4. [ADR-004: GPS Navigation의 적용 범위와 정밀 Localization 방향](ADR-004-gps-navigation-scope.md)
