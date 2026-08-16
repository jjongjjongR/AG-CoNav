# GLIM(+GPS 사후결합) 전체 파이프라인 실험 결과 — 미완료

## 전제
이 시뮬레이션의 LiDAR/IMU/GPS 센서 정의(agconav_description)에는
`<noise>` 블록이 전혀 없다 — 이 GPS는 잡음 없는 완벽한 위치를
발행한다. 그러므로 이 실험은 "실전의 노이즈 있는 GPS로 얼마나
개선되는가"가 아니라, **"GLIM의 위치추정 드리프트를 정밀한 위치
앵커로 억제했을 때, GLIM의 지도생성 파이프라인 자체가 제대로
작동하는가"**를 검증하는 것이다.

## 결과 요약: 목표 미달성

GLIM(CT+passthrough+pose_graph, 이 VM의 GPU 없는 환경에서 유일하게
쓸 수 있는 조합)이 이 4코어/5.8GB RAM VM에서 22508개 스캔 전체를
안정적으로 처리하지 못했다. 4가지 파이프라인 조합을 시도했으나 전부
실패:

| 시도 | 설정 | 결과 |
|---|---|---|
| 1차 | local+global mapping 전체 켬 | 12청크 재생은 완료했으나 OOM killer에 SIGKILL — 궤적 완전 손실 |
| 2차 | odometry-only(둘 다 끔) | 메모리는 안정, 하지만 궤적이 t=136.5s부터 발산(z=-102m) |
| 3차 | local mapping만 켬 | 메모리는 안정, 하지만 t=106.4s부터 발산(z=153m) |
| 4차 | 1차와 동일 + 메모리가드 | 가드로 33%까지 처리 후 안전 종료했으나 t=101.3s부터 발산(z=265m까지) |

"메모리를 아끼면 궤적이 발산하고, 메모리를 안 아끼면 OOM으로 죽는다"는
딜레마에 부딪혔고, 세션 시간 안에 근본 원인(CT odometry 초기화
파라미터가 이 bag의 실제 시작 조건과 안 맞을 가능성 등)을 해결하지
못했다. 상세 진단 기록은 `run_results/PROGRESS.md` "[새 세션]
GLIM(+GPS 사후결합) 전체 파이프라인 실험" 절 전체 참고.

## GPS 결합 방식 (설계·구현은 완료, 미실행)

GLIM 코어에는 GPS/GNSS 네이티브 지원이 없다. 별도 저장소
`koide3/glim_ext`에 `libgnss_global.so`(GNSS 기반 전역 제약)가
있으나, 공식 문서에 "half-baked code that may not be well-maintained
and not suitable for practical purposes"라고 명시돼 있어 채택하지
않았다. 지시사항의 폴백대로 **사후 결합(loose coupling)**을 설계:

```
P_raw(t)       : GLIM 원시 pose (traj_lidar.txt, TUM format, slerp/lerp 보간)
C_k            : GPS 앵커 시각 t_k에서의 보정 transform
                 translation = GPS_ENU(t_k) - P_raw(t_k).translation
                 rotation = identity (GPS는 orientation 정보가 없음)
C(t)           : 앵커 사이 SE3 보간(translation lerp, rotation slerp)
P_corrected(t) = C(t) @ P_raw(t)
```

GPS 위경도→ENU 변환은 `pymap3d`(WGS84 geodetic↔ENU), 기준점은 world
파일의 `<spherical_coordinates>`(lat=37.54233814881853,
lon=127.06050643805561, alt=15.4m). 구현: `run_results/glim_gps_correct.py`.
정상적인 GLIM 궤적이 없어 실행하지 못했다(발산한 궤적에 위치 보정만
얹어봐야 의미 있는 결과가 안 나오므로 시도 안 함).

## 4가지 지표

| 지표 | 값 | 비고 |
|---|---|---|
| **비행 소요시간** | **2254.09초 (≈37.57분)** | bag duration 실측(재비행 완료, `bags/velocity_4m_5mps_gps_attempt2`). 이전 GPS 없는 동일 경로/속도 실험(2258.8s)과 거의 일치 — 재현성 확인됨. |
| 커버리지 | 산출 못함 | 정상 GLIM 궤적이 없어 지도 자체를 못 만듦 |
| wheel FN% | 산출 못함 | 상동 |
| wheel 최대연결덩어리% | 산출 못함 | 상동 |
| leg 최대연결덩어리% | 산출 못함 | 상동 |

## 준비된 후속 자산 (다음 세션이 정상 궤적만 확보하면 바로 실행 가능)

- `bags/velocity_4m_5mps_gps_attempt2` — GPS 포함 velocity bag, 정상
  확보됨(`ros2 bag reindex` 완료, 재비행 불필요).
- `src/agconav_test_worlds/launch/experiment.launch.py` — GPS
  브리지(`/drone/gps`)와 bag 기록 토픽 추가 완료.
- `run_results/add_point_times_single.py`,
  `run_results/run_glim_chunked.sh` — 디스크 제약(원본+t필드본 동시
  보유 불가) 우회용 청크 단위 처리 파이프라인(mcap 파일 12개를
  하나씩 t필드 변환→재생→삭제).
- `run_results/glim_gps_correct.py` — GPS 사후결합(위 알고리즘).
- `run_results/glim_gps_build_map.py` — 보정된 궤적으로 원본 스캔을
  map 좌표로 옮겨 셀별 단순평균 누적(칼만필터 아님, 지시사항대로
  drone_elevation_mapper.py 미사용).
- `run_results/glim_gps_metrics.py` — 커버리지/wheel FN%/wheel·leg
  최대연결덩어리%(사용자 지정 알고리즘 그대로) 계산.

## 다음 세션이 우선 시도해볼 것

1. `glim_config/config_odometry_ct.json`의 초기화/등속 사전확률
   파라미터(`constant_velocity_inf_scale` 등)를 이 bag의 실제 시작
   속도 조건에 맞게 재조정 — 기존 주석이 "8m/s로 순항 중 시작 가정"을
   전제하는데 이번 5m/s bag의 실제 시작 조건과 다를 가능성.
2. 그래도 발산하면 `config_odometry_cpu.json`(VGICP+IMU 타이트
   커플링)로 전환 — 이번 bag은 IMU 회전이 정상이라 이론상 가능(단
   sub_mapping/global_mapping도 대응 config로 함께 바꿔야 할 수
   있음, 미검증).
3. 궤적이 안정된 조합을 찾으면, 청크+메모리가드 파이프라인(이번
   세션에서 검증된 안전장치)을 그대로 재사용해 전체를 처리.
