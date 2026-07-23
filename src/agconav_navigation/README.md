# agconav_navigation  (담당: 이수빈)  — 모듈 C (지상 Nav2 이동)

wheel·leg를 공통 Nav2로 각자 주행맵 위에서 목표까지 이동시키는 패키지.

- 입력: /wheel/nav_map·/leg/nav_map(모듈 F), 위치추정 TF(모듈 B), /X/points.
- 출력: /X/cmd_vel.  장애물: Voxel Layer로 PointCloud2 직접(LaserScan 없음).
- 로봇별 차이: namespace·footprint만. Nav2 설정(planner/controller/BT)은 공통 하나.
- 범위 밖: 지도 생성(A/F), 위치추정(B), 지도 누적(D), 병합(E).
