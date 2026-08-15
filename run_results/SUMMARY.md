# SUMMARY — 84m + 방법B(GT pose + GICP 정합) 실험

## 목적

GLIM 자체 위치추정(odometry)은 쓰지 않는다. GT(시뮬레이터가 아는 실제 물리
위치)를 그대로 신뢰하고, GLIM이 내부적으로 쓰는 `small_gicp` 라이브러리의
point cloud 정합(registration) 능력만 빌려 elevation map의 노이즈를 줄이는
것이 목적("방법B"). GLIM 전체(odometry/pose-graph)는 이 목적에 맞지 않는다는
것을 이전 조사에서 확인했다(외부 pose 주입/정합 전용 모드 비공식 지원,
GitHub 이슈 #193 미해결로 확인) — 그래서 GLIM을 통째로 쓰는 대신 `small_gicp`
을 직접 호출하는 스크립트(`build_methodB_cloud.py`)를 새로 작성했다.

## 방법

1. **비행**: `path_100x100.yaml`(84m 고도, 8m/s, 32m 간격, 10웨이포인트,
   기존 검증된 값 그대로) + `flight:=velocity`(실제 추력 비행) +
   `Seongdong_gu_100x100_dynamic` 월드. bag: `/drone/points`, `/drone/imu`,
   `/tf`, `/tf_static`, `/drone/path_status`만 기록(module_a/f는 라이브로
   안 돌림 — 방법B는 bag만 있으면 오프라인으로 처리 가능해서 불필요한 I/O
   부하를 피함).
2. **방법B cloud 생성**(`run_results/build_methodB_cloud.py`): bag의
   `/tf`(map→drone/base_link, 물리엔진이 실제로 계산한 진짜 위치) +
   `/tf_static`(base_link→os1_lidar)을 tf2 Buffer에 그대로 먹여 매 LiDAR
   스캔 시각마다 GT pose를 오프라인으로 조회. 그 GT pose를 GICP 초기값으로,
   직전 6개 스캔(월드 프레임, 이미 방법B로 정렬된 것)을 타겟 삼아
   `small_gicp.align(..., registration_type='GICP', downsampling_resolution=0.3,
   max_correspondence_distance=1.0)`로 미세정합 → 정합된 pose로 그 스캔을
   누적. GICP가 수렴 실패하면 GT pose 그대로 사용(안전 쪽 fallback).
   같은 스캔들을 GT pose만으로(정합 없이) 누적한 baseline cloud도 동시에 생성.
3. **채점**: 기존(무수정) Module A(`drone_elevation_mapper`) → Module F
   (`terrain_feature_calculator` + `traversability_verdictor` wheel/leg)
   파이프라인에 각 cloud를 `feed_cloud.py`로 그대로 흘려보내고,
   `capture_nav_fn.py`로 GT 통과가능 셀(`gt_traversable.py`: 건물 풋프린트
   밖 + 인접 셀 높이차 wheel<0.08m/leg<0.15m) 대비 파이프라인이 "막힘"으로
   오판한 비율(FN%)을 계산. 이 FN% 정의는 이번에 새로 정한 것이 아니라
   기존에 확정된 정의를 그대로 재사용했고, RESULTS.md의 옛 "초과 %" 지표를
   재사용하지 않고 이번 정의로 baseline도 다시 계산했다(분모가 다름).

## 결과

| 방식 | 위치 정합 | wheel FN% | leg FN% | 결과 |
|---|---|---|---|---|
| 84m GT (기존, 정합 없음) | GT pose 그대로 | 23.94% | 12.15% | - |
| 84m GT + 방법B(GICP 정합) | GT pose + 정합 | **20.10%** | **11.89%** | wheel 개선(-3.84pp, 상대 16.0%↓), leg 거의 무변화(-0.26pp, 상대 2.1%↓) |

(GT 통과가능 셀 618,378개 기준, 둘 다 동일 — GT는 정합과 무관하게 고정.)

**부수 지표(전체 셀 기준, GT 통과가능 셀로 한정하지 않은 원시 단차 통계)**:

| 지표 | baseline | 방법B | 방향 |
|---|---|---|---|
| step 중앙값 | 0.112 m | 0.123 m | 악화 |
| step p95 | 1.619 m | 2.663 m | 크게 악화 |
| step max | 89.04 m | 82.63 m | 소폭 개선 |
| 커버리지(전체 격자 중 유효 셀) | 44.13% | 47.17% | 개선 |
| GT 통과가능 셀 중 미측정(unknown) 비율 | 66.39% | 72.93% | 악화 |

## 해석 — 정합이 노이즈를 줄였는가?

**부분적으로만, 그리고 대가가 있다.** 사전에 세운 가설("정합이 이미 완벽한
GT 위 점들 사이에서는 맞출 게 별로 없어서 효과가 미미할 것") 은 wheel FN%
에는 안 맞았고(16% 상대 개선으로 무시할 수준이 아님) leg FN%에는 거의
맞았다(2%만 개선). 그런데 전체 셀 기준 원시 단차 통계(중앙값·p95)는
오히려 **악화**됐다 — 이 세 가지를 함께 보면 다음과 같이 해석된다:

- **FN%는 GT가 "쉬운"(평탄, 통과가능) 곳만 본다.** 이런 곳은 스캔끼리
  겹침이 좋고 형태가 단순해 GICP가 잘 수렴한다(전체 스캔의 94.5%가
  수렴 성공) — 그 결과 wheel 기준(0.08m, 노이즈 크기와 비슷한 아주 좁은
  문턱)을 넘던 셀 중 일부가 정합으로 노이즈가 줄어 문턱 아래로 내려와
  "막힘" 오판이 줄었다. leg 기준(0.15m)은 이미 노이즈보다 두 배 넓어
  개선 여지 자체가 작았다(RESULTS.md에 이미 기록된 "wheel 기준이 노이즈
  선상에 있다"는 관찰과 일치).
- **하지만 원시 통계(p95: 1.62→2.66m, 거의 65% 악화)는 GICP가 일부
  스캔에서 오히려 더 크게 어긋났다는 뜻이다.** GICP 실패율 5.5%(281/5169)
  자체는 GT로 안전하게 대체됐지만, "성공"으로 표시된 정합 중에서도
  형태가 빈약한 지역(예: 84m 고도에서 평탄한 지붕/지면 — RESULTS.md가
  이미 GLIM 결과 분석에서 지적한 "수평/yaw 방향을 구속할 수직 구조물
  부족"과 같은 문제)에서는 GICP가 그럴듯하지만 틀린 해로 수렴했을
  가능성이 높다. 이런 지역은 GT가 "통과가능"으로 보지 않는 셀(건물
  근처, 단차가 큰 지역)에 몰려 있어 FN% 계산에는 안 잡히고 p95/max에만
  잡힌 것으로 보인다.
- **커버리지-정확도 트레이드오프도 있다**: GT 통과가능 셀 중 미측정
  비율이 66.4%→72.9%로 늘었다(반면 전체 격자 커버리지는 44.1%→47.2%로
  늘어 방향이 반대다 — 정합이 점을 넓게 퍼뜨리면서 일부 영역은 새로
  채우고 일부 영역(GT 통과가능 셀이 몰린 평탄 구간)은 오히려 셀당 점
  밀도가 낮아져 "측정 안 됨" 문턱을 못 넘긴 것으로 추정). `downsampling_
  resolution=0.3m`으로 다운샘플한 것도 한 원인일 수 있다.

**결론**: 방법B는 "이미 쉬운 곳을 더 쉽게" 만드는 효과(wheel FN% 개선)는
있지만, "어려운 곳을 더 어렵게" 만드는 부작용(원시 노이즈 지표 악화,
미측정 비율 증가)도 함께 가진다. GT 신뢰 전제 자체는 지켜졌다(GICP 실패
시 안전하게 GT로 폴백) — 하지만 이번 파라미터(직전 6스캔 타겟, GICP
downsampling 0.3m)로는 순수 이득이라 부르기 어렵다.

## 산출물

- `run_results/build_methodB_cloud.py` — 방법B/baseline cloud 생성 스크립트(재사용 가능)
- `run_results/run_84m_velocity_monitored.sh` — 84m velocity 비행 실행+감시 스크립트
- `run_results/84m_baseline_fn.json`, `run_results/84m_methodB_fn.json` — 채점 결과 원본
- `run_results/maps_84m/{baseline,methodB}_elevation_map/` — 각 방식의 실제 저장된 elevation_map(mcap)
- bag(`bags/velocity_84m`, 5.1GB)과 point cloud(`run_results/clouds/*.npy`, 166MB)는
  둘 다 스크립트로 재현 가능(84m 비행은 ~3분, cloud 생성은 ~46초)해서 git에는
  올리지 않음. bag은 디스크 확보를 위해 삭제, cloud는 로컬에 보존.

## 알려진 버그(이번에 발견, 기록만 하고 재발 방지용으로 남김)

`run_84m_velocity_monitored.sh`의 정지(stall) 감지 로직이 "wp N/N(마지막
웨이포인트)에서 값이 안 바뀌는 것"을 정지로 오판해 실제로는 정상 완주
(167초 만에 10/10 도달 + `path_status=True` 발행)한 비행을 5분 뒤
불필요하게 SIGTERM으로 중단시켰다. bag은 `ros2 bag reindex`로 완전히
복구됐고(5,169 스캔, TF 25,987개, 전부 정상) 재비행은 필요 없었지만,
다음에 이 스크립트류를 쓸 때는 "마지막 웨이포인트 도달 후에는 진행률
정지를 정지 신호로 안 보고 path_status만 신뢰"하도록 고쳐야 한다.
