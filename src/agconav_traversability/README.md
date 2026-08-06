# agconav_traversability  (담당: 이종헌)  — 모듈 F (지형 주행성 분석)

모듈 A의 드론 2.5D 지도에서 로봇별(wheel/leg) 주행 가능 맵을 분리하는 패키지.

- 입력: /drone/elevation_map + 로봇별 통과 기준(최대 경사·최대 단차).
  장애물 높이는 2.5D 지도에서 단차와 같은 값이라 따로 쓰지 않는다.
- 출력: /wheel/nav_map, /leg/nav_map (2D OccupancyGrid, frame=map, 0.10 m/cell).
- wheel/leg 같은 분석 구조, 통과 기준 파라미터만 다름. (낮은 장애물: wheel=막힘 / leg=통과)
- 범위 밖: 지도 생성(A), Nav2 이동(C), 지상 지도 누적(D), 병합(E), foothold 계획(향후 확장).

## 노드

| 노드 | 하는 일 |
| --- | --- |
| `terrain_feature_calculator` | 완료 신호를 받으면 드론 지도로 셀별 경사·단차를 1회 계산 → `/terrain/features` |
| `traversability_verdictor` | 특성 지도를 통과 기준과 비교 → `/X/nav_map` + `/X/nav_map_status` (wheel·leg 각각 실행) |
| `map_save_coordinator` | 두 상태가 모두 True면 `/map_saver/save_map`을 wheel → leg 순서로 호출 |
| `map_saver_server` | nav2_map_server 제공. 지정 토픽을 받아 `.yaml`+`.pgm`으로 저장 |

**경사와 단차를 다르게 재는 이유**: 단차는 인접 셀과의 최대 높이차, 경사는
`slope_window` 반경 중심차분 기울기다. 한 셀 차이로 경사를 재면 0.10 m 턱이 곧
45°가 되어 두 기준이 같은 값이 되고, wheel(20°/0.08 m)·leg(30°/0.15 m) 판정이
갈리지 않는다.

## 실행

```bash
ros2 launch agconav_traversability traversability.launch.py
```

통과 기준은 `config/traversability_wheel.yaml`, `config/traversability_leg.yaml`에서 튜닝한다.
