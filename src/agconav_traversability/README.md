# agconav_traversability  (담당: 이종헌)  — 모듈 F (지형 주행성 분석)

모듈 A의 드론 2.5D 지도에서 로봇별(wheel/leg) 주행 가능 맵을 분리하는 패키지.

- 입력: /drone/elevation_map + 로봇별 통과 기준(최대 경사·최대 단차·최대 장애물 높이).
- 출력: /wheel/nav_map, /leg/nav_map (2D OccupancyGrid, frame=map, 0.10 m/cell).
- wheel/leg 같은 분석 구조, 통과 기준 파라미터만 다름. (낮은 장애물: wheel=막힘 / leg=통과)
- 범위 밖: 지도 생성(A), Nav2 이동(C), 지상 지도 누적(D), 병합(E), foothold 계획(향후 확장).
