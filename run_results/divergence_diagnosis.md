# GLIM odometry 발산 원인 진단 (1단계, ①~⑤ 종합)

조건: 5m AGL / 스트립간격 4m / 5m/s, bag `bags/velocity_4m_5mps_gps_attempt2`
(GPS 포함, 재사용, 재비행 없음). GLIM config는 기존 그대로
(CT odometry + passthrough sub_mapping + pose_graph global_mapping,
`glim_config/`). **파라미터 튜닝은 하지 않았다** — 아래는 전부 진단 결과다.

診단 실행 범위: 전체 bag(2254s, 22508스캔) 대신 **chunk 0(190.5s,
1899스캔, t=8.9~198s)만 사용**했다. 근거: 이전 세션(같은 브랜치 이전
커밋 `5a8559d`)이 4가지 파이프라인 조합(풀 파이프라인, odometry-only,
local-only, 풀+메모리가드) 전부에서 **t≈101~136s 부근부터 이미
"물리적으로 불가능한" 절대값**(z가 -100m대 등)으로 발산하는 것을
이미 확인해뒀다 — chunk 0(0~190s)이 이 구간을 충분히 포함한다. 이번
세션은 그 발산을 GT와 정량 대조해 "언제·어떻게" 시작되는지 정밀화하는
것이 목적이었다.

## 요약 결론

**발산은 이전에 생각했던 것보다 훨씬 일찍(t≈15~17s, 비행 시작 직후)
시작된다.** 이전 세션은 "절대 좌표값이 물리적으로 말이 안 되게 커지는
시점"(t≈101~136s)을 발산 시작으로 봤지만, 그건 오차가 ~100초 동안
누적된 뒤에야 육안으로 명백해진 시점일 뿐이다. **GT와 정렬해 유클리드
오차를 실측하면, 오차는 비행 시작 후 불과 6~8초 만에(t≈15~17s)
이미 뚜렷하게 커지기 시작한다.** 이 시점은:
- 드론이 **정지(hover) 상태에서 전진 비행으로 전환되는 바로 그
  시점**과 거의 정확히 일치한다(아래 ①② 참고).
- 디스큐를 켜든 끄든(⑤) **거의 동일**하다(t=15.7s vs 15.9s) — 디스큐는
  발산의 "양상"은 크게 바꾸지만 "시작 시점"은 못 바꾼다.
- 타임스탬프 이상(③)이나 외부보정 오류(④)는 **모두 배제됐다**(둘 다
  깨끗함, 아래 참고).

즉 가장 유력한 근본원인은 **CT 오도메트리가 "정지→전진" 전환 구간을
제대로 못 넘긴다**는 것이고, 디스큐 설정은 그 실패를 더 악화(또는
다른 양상으로 변형)시키는 2차 요인으로 보인다.

## ① GT 궤적 vs GLIM 궤적 비교 / ② 발산 시작 시점 특정 (통합 서술)

방법: chunk 0을 라이브 `glim_rosnode`(config 그대로, 메모리 가드
4600MB 적용 — 이번엔 발동 안 함, chunk가 짧아 정상 종료)로 처리해
`odom_imu.txt`를 얻고, bag의 `/tf`(map→drone/base_link, GT)와
header.stamp 축으로 정렬했다. GLIM의 odom/map 좌표계는 **첫 스캔을
원점(0,0,0)으로 잡는 자체 좌표계**라 GT(월드 절대좌표)와 상수
오프셋+회전이 있다 — 초반 5초(발산 전이라고 볼 수 있는 구간)만으로
Umeyama(스케일 고정) 정합한 뒤 전체 궤적에 적용했다
(`run_results/diag_scripts/compare_gt_glim.py`, `ate_align.py`의
Umeyama를 재사용).

결과(`run_results/glim_diag_dumps/run1_full_pipeline/gt_vs_glim_aligned.png`):
- t=8.9~13.6s: 오차 0.13~6.17m(초반이라 정합 구간과 겹쳐 작음, 정상).
  **이 구간 동안 GT 위치가 (-31.60,-159.09,3.06)에서 거의 안 움직인다**
  — 드론이 호버링 중(z=3.06m, 아직 목표 5m AGL까지 못 올라간 상태,
  상승 중으로 추정).
- t≈15.7s(자동 탐지, 오차 변화율이 직전 구간 중앙값의 5배를 처음
  넘는 지점): 오차가 본격적으로 커지기 시작.
- t=39.2s: 오차 81m, z추정치 84.95m(고도 84m는 5m AGL 실험 조건과
  전혀 안 맞음 — 이미 이 시점에 명백히 망가진 상태).
- t=150.1s: 최대오차 306m(정합 전 raw) / 247m(정합 후).
- 궤적 모양(PNG 왼쪽 패널) 자체가 GT의 단순한 왕복 스트립 패턴과 달리
  **혼란스러운 스크리블(zigzag) 형태** — 특정 방향으로 일관되게
  드리프트하는 게 아니라 매 스캔마다 방향이 요동친다.

**결론: 발산은 "완만하게 커지다가"가 아니라, 비행 시작 후 불과
6~8초 만에 급격히 시작된다.** 경로 계획상 이 시점은 **아직 첫 번째
턴/코너에도 도달하기 전, 이륙 후 첫 전진 명령(첫 웨이포인트 진행)
직후**다 — 즉 "코너에서 발산"이라는 가설은 이번 데이터로는 기각되고,
"정지→전진 전환에서 발산"이라는 가설로 좁혀진다.

## ③ LiDAR/IMU 타임스탬프 동기화 확인 — 이상 없음 (원인에서 배제)

`run_results/diag_scripts/check_timestamps.py`로 bag 전체(22508
`/drone/points`, 224866 `/drone/imu`)의 header.stamp를 검사했다
(`run_results/logs/check_timestamps.log`):
- **역전(reversal) 0건** — 두 토픽 모두 완벽하게 단조증가.
- 간격 통계: `/drone/points` mean_dt=0.100009s(기대 0.1s), `/drone/imu`
  mean_dt=0.010010s(기대 0.01s) — 설계값과 정확히 일치, use_sim_time
  기준 어긋남 없음.
- 이상(2배 이상 간격) 총 14건, 전부 **t+593s 이후**(bag 후반부)에만
  존재하고 최대 간격도 0.26s로 크지 않음.
- **발산 구간(t+90~150s, ②의 t=15~40s와는 별개로 이전 세션이 지목한
  절대값 기준 발산 구간)에는 이상 0건.**

**결론: 타임스탬프 동기화는 발산의 원인이 아니다.** 이는 ②에서
새로 좁힌 발산 시작 시점(t=15~17s)에도 그대로 적용된다 — 그 구간
근처에도 타임스탬프 이상은 전혀 없었다.

## ④ LiDAR-IMU 외부보정(extrinsic) 확인 — 이상 없음 (원인에서 배제)

0단계가 성공해 `imu_validator`를 빌드했지만, 실제 실행 검증(라이선스
확인 및 로드 성공까지만, `glim_ext_feasibility.md` 참고) 대신 **더
직접적인 방법인 수치 재계산**으로 검증했다(둘 다 유효하지만, 시간
절약을 위해 후자만 수행 — imu_validator를 이 특정 bag에 대해 실제
구동해보는 것은 다음 세션에서 필요시 추가 가능).

`agconav_test_worlds/models/agconav_drone_dynamic/model.sdf`(SDF 1.6,
`relative_to` 미사용 → 모든 `<link><pose>`는 모델 프레임 기준)를 직접
읽어 확인:
- IMU 센서: `<sensor name="drone_imu_sensor">` pose `(0,0,-0.04,0,0,0)`,
  부모 링크가 `X3/base_link`(모델 원점과 동일 pose).
- LiDAR: `<link name="os1_lidar"><pose>0 0 -0.175406 0 1.5708 0</pose>`
  (모델 프레임 기준, `os1_lidar_mount`와 별개 링크라 오프셋이 이중으로
  누적되지 않음 — SDF 1.6에서 `<pose>`는 기본적으로 모델 프레임 기준이지
  부모 링크 기준이 아니므로, 언뜻 헷갈릴 수 있는 2단 링크 구조지만
  버그 없음을 확인).

손으로 T_lidar_imu = T_lidar_base · T_base_imu 를 계산(회전 Ry(-90°)
적용)한 결과:
- translation: (-0.135406, 0, 0)
- quaternion: (qx=0, qy=-0.7071068, qz=0, qw=0.7071068)

`glim_config/config_sensors.json`의 `T_lidar_imu` 값과 **소수점까지
정확히 일치**한다.

**결론: LiDAR-IMU 외부보정은 정확하다. 발산의 원인이 아니다.**

## ⑤ per-point timestamp / 디스큐 설정 확인 및 on/off 비교

현재 설정(`config_sensors.json`): `autoconf_perpoint_times: true`,
`perpoint_relative_time: true`, `perpoint_time_scale: 1.0`,
`global_shutter_lidar: false`(디스큐 켜짐). 이 LiDAR가 point-level
타임스탬프 필드를 안 주므로, bag을 `add_point_times_single.py`로
전처리해 `t` 필드(스캔 배열의 방위 열(column) 인덱스 기반,
`t=col/width*0.1`)를 추가한 뒤 GLIM에 먹인다 — 이는 우리가 칼만필터
파이프라인(변형 ④, PROGRESS.md 참고)에서 썼던 "방위각 기반 12구간
근사" 디스큐와 **다른 가정**을 쓴다: 칼만필터 쪽은 실제 atan2 방위각을
12개 구간으로 나눠 구간별 TF를 따로 조회하지만, 이 column-index 근사는
"발사 순서 = 열 인덱스"라는 organized-cloud 구조를 그대로 믿고 매
점마다(사실상 무제한 세분화) 시간을 선형 배분한다 — 이론적으로는 더
정밀하지만, 유효 반사가 방위 한 구획에 몰려있다는 이 LiDAR의 특성과
맞물리면 다르게 틀어질 여지가 있다(config 주석이 이미 이 문제를
지적하고 있음).

**비교 실행**: 같은 chunk 0을 (A) 기존 설정 그대로, (B)
`global_shutter_lidar: true`로만 바꿔(디스큐 완전히 끔,
`glim_config_nodeskew/`) 각각 1회 실행.

| | 발산 시작(자동탐지) | 최대오차 | 궤적 양상 |
|---|---|---|---|
| A. 디스큐 ON(기존) | t=15.7s | 306m(raw)/247m(정합) | **혼란스러운 고빈도 스크리블**, 오차가 100~250m 범위에서 계속 요동, 회복 기미 없음 |
| B. 디스큐 OFF | t=15.9s | 135m | 비교적 **부드러운 단일 드리프트**(전진을 거의 못 따라가고 제자리 근처에서 맴돌며 z가 -84m까지 단조 하강했다가 회복), 오차 곡선이 완만한 삼각형 형태 |

(`run_results/glim_diag_dumps/run1_full_pipeline/gt_vs_glim_aligned.png`,
`run_results/glim_diag_dumps/run2_nodeskew/gt_vs_glim_aligned.png`)

**해석**: 발산 **시작 시점은 디스큐 on/off와 무관하게 거의 동일**
(15.7s vs 15.9s, 0.2초 차이) — 디스큐가 발산의 **1차 방아쇠는 아니다**.
그러나 발산 **이후의 양상(심각도·형태)은 확실히 다르다** — 디스큐
ON이 OFF보다 최대오차가 2배 이상 크고(306m vs 135m) 훨씬 더
불안정(고빈도로 요동)하다. 지시사항의 "다르면 원인이 상당히 좁혀진다"
기준으로 보면: **디스큐(우리의 column-index 근사)는 발산을 일으키는
원인이 아니라, 이미 시작된 발산을 증폭시키는 2차 요인**으로 좁혀진다.

## 종합 진단 및 우선순위

| 순위 | 후보 원인 | 근거 | 상태 |
|---|---|---|---|
| **1순위** | **CT 오도메트리의 "정지→전진 전환" 처리(초기화/등속 사전확률)** | 디스큐 on/off 무관하게 발산 시작 시점이 거의 정확히 호버링 종료·전진 시작 시점과 일치(t≈15~17s). `config_odometry_ct.json`의 `constant_velocity_inf_scale`(1e0으로 완화됨) 주석이 "이 bag은 이미 8m/s로 순항 중일 때 시작한다"고 명시하는데, 실측 GT는 **t=8.9~13.6s 동안 정지(hover)해 있다가 그제서야 움직이기 시작** — 주석의 전제 자체가 이 bag(velocity_4m_5mps_gps_attempt2, 5m/s)의 실제 초기 거동과 다르다(다른 실험/속도 조건에서 복사된 설정일 가능성). | 미검증 가설이지만 정황증거 강함 |
| 2순위 | per-point 디스큐(column-index 근사)의 정확도 | ⑤에서 on/off 비교로 발산의 **심각도**(2배+)와 **양상**(혼란스러움 vs 매끄러움)이 확실히 달라짐을 실측 확인. 근본원인은 아니지만 개선하면 발산의 크기를 줄일 여지가 있다. | 실측 확인됨(2차 요인) |
| 배제 | 타임스탬프 동기화(③) | 전체 bag 22508+224866개 메시지 검사, 역전 0건, 발산구간 근처 이상 0건 | 배제 |
| 배제 | LiDAR-IMU 외부보정(④) | model.sdf 실측값으로 직접 재계산해 config 값과 소수점까지 일치 확인 | 배제 |

## Phase 2(파라미터 튜닝)를 위한 제안

1. **최우선**: `config_odometry_ct.json`의 `constant_velocity_inf_scale`을
   이 bag(5m/s, 호버 시작)의 **실제** 초기 속도 프로파일에 맞게
   재검토. 구체적으로: (a) bag 초반 IMU/GT로 실제 정지 구간 길이와
   가속 프로파일을 측정하고, (b) 정지 구간에서는 강한 등속 사전확률
   (원래 문서 기본값 1e3에 가까운 값)이 오히려 유리할 수 있으므로,
   "8m/s 순항 시작" 전제로 낮춰둔 1e0이 이 bag엔 안 맞을 가능성을
   검토. 시간이 된다면 1e0/1e2/1e3 간단 스윕으로 발산 시작 시점이
   늦춰지는지만 먼저 확인해볼 것을 권한다(본격 파라미터 최적화는 별도).
2. **2순위**: 디스큐를 끈 상태(B, `global_shutter_lidar: true`)가
   켠 상태보다 최대오차가 작았다는 이번 실측 결과를 감안해, Phase 2
   시작 시 **디스큐를 일단 꺼둔 상태**를 새 기준선으로 삼는 것을
   고려. 그 위에서 1순위 파라미터를 튜닝해 발산이 억제되면, 그 다음
   단계로 디스큐(우리의 column-index 근사, 또는 칼만필터 파이프라인이
   썼던 12구간 방위각 근사)를 다시 켜고 정밀도를 높이는 방향으로
   순서를 잡는 게 안전하다 — 두 요인을 동시에 바꾸면 이번 세션처럼
   원인 분리가 어려워진다.
3. `smoother_lag`(현재 1.0s)·`max_correspondence_distance`(현재 2.0)는
   이번 진단 범위 밖이라 손대지 않았지만, 1순위 튜닝 후에도 발산이
   남으면 다음으로 검토할 후보로 기록해둔다(정지→전진 전환 구간에서
   iVox 맵이 정지 스캔들로만 구성돼 있다가 갑자기 큰 모션이 들어오면
   `max_correspondence_distance`가 너무 좁아 대응(correspondence)을
   못 찾을 가능성).

## 재현 방법 / 산출물 위치

- 진단 스크립트: `run_results/diag_scripts/check_timestamps.py`,
  `run_results/diag_scripts/compare_gt_glim.py`.
- GLIM 궤적 덤프: `run_results/glim_diag_dumps/run1_full_pipeline/`
  (디스큐 ON, 기존 config), `run_results/glim_diag_dumps/run2_nodeskew/`
  (디스큐 OFF, `glim_config_nodeskew/`).
- 타임스탬프 진단 로그: `run_results/logs/check_timestamps.log`.
- glim_ext 조사: `run_results/glim_ext_feasibility.md`.
- 두 실행 모두 **chunk 0(bag 앞 190.5초)만 사용** — 발산이 이미 이
  구간 안에서 명확히 관찰돼 전체 bag(2254초, 처리에 5~6시간 필요,
  이전 세션에서 실측됨)을 다시 돌릴 필요가 없었다.
