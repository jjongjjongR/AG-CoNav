# agconav_drone  (담당: 홍연주)  — 모듈 A (드론 지도 생성)

드론을 pose 경로로 kinematic 이동시키고, 하향 OS1-32 점군으로 드론 2.5D 지도를 만드는 패키지.

- 이동: 경로 YAML(탐지 고도 84m, 하향 스캔) -> /drone/cmd_pose, 드론 위치 TF(map -> drone/base_link).
- 지도: grid_map + elevation_mapping -> /drone/elevation_map (layer=elevation, 0.10 m/cell, frame=map), 저장.
- 범위 밖: 자율비행·비행동역학, 주행성 분석(모듈 F), 지상 지도 누적(모듈 D), 병합(모듈 E).
