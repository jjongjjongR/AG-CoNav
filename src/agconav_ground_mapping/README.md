# agconav_ground_mapping  (담당: 채현우)  — 모듈 D (지상 지도 누적)

wheel·leg가 이동하며 관측한 3D 점군을 2.5D elevation 지도로 누적하는 패키지(모듈 A 구조 재사용).

- 입력: /wheel/points·/leg/points + 관측 시각의 위치·TF(모듈 B).
- 출력: /wheel/elevation_map·/leg/elevation_map (layer=elevation, 0.10 m/cell, frame=map), 저장.
- 범위 밖: 위치추정(B), 주행(C), 병합(E). 지상 독립 SLAM 안 함.
