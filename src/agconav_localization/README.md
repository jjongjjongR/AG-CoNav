# agconav_localization  (담당: 이수빈)  — 모듈 B (지상 위치추정)

wheel·leg의 GPS+odometry를 융합해 공통 map 좌표로 정렬하는 패키지(robot_localization: EKF + navsat).

- 입력: /X/gps, /X/odom.  출력: map -> X/odom TF, 필터링된 odom.
- 규칙: ground-truth pose 사용 금지. map -> odom TF는 이 패키지만 발행(TF 소유권).
- 범위 밖: 드론 위치(A), 주행(C), 지도 누적(D).
