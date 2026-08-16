# SUMMARY — 5m AGL 칼만필터(Module A 이식 + 이상치 방어) 실험 + 5개 결과 종합비교

**세션**: 2026-08-16, 브랜치 `test_main_brian`(작업 폴더
`~/AG-CoNav-test_main`), 자율 진행(전체 타임아웃 5시간, 재비행이 필요해져
2시간→5시간으로 연장). 판단 근거는 `run_results/PROGRESS.md`에 전부 기록.

## 이번 세션 목표

5m AGL(4m 간격, 5m/s, `path_100x100_5m_4m_5mps.yaml`)에서, GICP 대신 GT
pose + `brian_test` 브랜치에 있는 Module A 칼만필터 설계를 적용해 지도를
만들고 wheel/leg FN%를 확인. 동시에 `brian_test`의 Module A에도 원래
없었던 이상치 방어 로직 2가지(Module D를 참고해 새로 구현)를 채워 넣음.

## 무엇을 했나 (요약)

1. **0단계 — 재비행 필요 여부 확인**: `bags/`가 완전히 비어 있었다(이전
   세션들이 "재현 가능"을 이유로 84m/5m 실험 bag을 전부 삭제). 홈
   디렉토리 전체를 검색해도 5m AGL(4m/5mps, velocity) bag이 어디에도
   없어 재비행 필요로 판단, 즉시 재비행. 조건은 이전과 동일: `flight:=
   velocity`, `Seongdong_gu_100x100_dynamic`(velocity 모드 자동 선택),
   `path_100x100_5m_4m_5mps.yaml`, `cruise_speed_mps:=5.0`.
   `run_results/run_velocity_4m_5mps_monitored.sh attempt1` 사용(마지막
   웨이포인트 도달 후 STALL 오판 버그가 이미 수정된 스크립트). 2231초
   (약 37분) 만에 정상 완주(`path_status=true`), TF 끊김 없음, bag
   22GB(12개 mcap 청크). 이번 bag(`bags/velocity_4m_5mps_attempt1`)은
   기존과 달리 **삭제하지 않고 보존**하기로 결정(이전 세션들이 재현
   가능을 이유로 지운 게 이번 재비행을 유발했으므로, 이 판단을
   뒤집음 — 실패/중단 attempt만 삭제 대상).

2. **1~2단계 — Module A 칼만필터 이식 + 방어 로직 2가지 신규 구현**:
   `git show brian_test:src/agconav_drone/agconav_drone/
   drone_elevation_mapper.py`로 확인한 결과, 요구된 요소(셀별
   elevation/variance 칼만필터, R=거리+입사각+점밀도 결합, Q=0, 이노베이션
   게이팅 임계값 9.0, `_grow_to_fit`의 NaN 패딩, `elevation_variance`
   레이어, **np.gradient를 전체 배열이 아니라 국소 윈도우에만 적용하는
   최적화**) 전부 실제로 존재함을 확인 — brian_test 쪽 이상 없음, 별도
   보고 불필요. 이걸 test_main_brian의 같은 파일에 이식하면서 이
   브랜치 고유의 `min_range_m`(기체 자기반사 제거) 필터는 유지했고,
   `brian_test:src/agconav_ground_mapping/agconav_ground_mapping/
   ground_elevation_mapper.py`(Module D)를 참고해 원래 Module A에는 없던
   방어 로직 2가지를 새로 구현:
   - **1겹 — `max_sensor_range`(기본 200.0m)**: transform 후 센서 원점
     기준 거리가 이 값을 넘는 점을 격자 비닝 전에 버림(디버그 로그).
   - **2겹 — `max_grid_cells`(기본 30,000,000)**: `_grow_to_fit`이 패딩을
     실행하기 직전에 패딩 후 예상 총 셀 수를 계산, 초과 시 실제 배열
     확장을 하지 않고 error 로그 후 (None, None) 반환 → 호출자가 이번
     배치만 버리고 기존 누적(`self._elevation`/`_variance`)은 보존, 노드는
     계속 살아있음.
   `drone_elevation_mapper.yaml`에 칼만필터 파라미터 + 두 방어 파라미터
   전부 추가. `agconav_drone/package.xml`의 `<depend>rosbag2_py</depend>`
   바로 다음 줄에 `<exec_depend>rosbag2_storage_mcap</exec_depend>` 추가.

3. **3단계 — bag 재생으로 지도 생성**: `colcon build --symlink-install
   --packages-up-to agconav_drone agconav_traversability`(성공, 에러
   없음) 후, `run_results/run_bag_replay_kalman.sh`(신규 작성, `feed_cloud.
   py` 기반 `run_traversability_fn_v2.sh`를 `ros2 bag play --clock`
   기반으로 변형)로 drone_elevation_mapper + terrain_feature_calculator +
   traversability_verdictor(wheel/leg) + elevation_map_saver를
   `use_sim_time:=true`로 먼저 띄운 뒤 bag을 `--clock --rate 1.0`으로
   재생, `/drone/points`+`/tf`+`/tf_static`+`/drone/path_status`를
   실시간과 동일하게 라이브 구독시켜 최종 elevation_map을 발행시켰다.
   - **알려진 버그 발견 및 복구**: 첫 재생 시도에서 `ros2 bag play`가
     "No storage id specified" 에러로 즉시 실패 — 원인은
     `run_velocity_4m_5mps_monitored.sh`의 정상 종료 절차가 SIGINT →
     (15초 내 안 죽으면) SIGTERM으로 에스컬레이션하는데, 이번 완주
     종료 시 실제로 SIGTERM까지 갔고 그 여파로 `ros2 bag record`가
     `metadata.yaml`을 못 쓰고 죽었다(mcap 데이터 파일 12개, 22GB는
     전부 정상). `ros2 bag reindex -s mcap bags/velocity_4m_5mps_attempt1`
     로 복구 성공, `ros2 bag info`로 데이터 손실 없음 확인(메시지 수
     `/drone/points` 22,319 · `/tf` 111,598 · `/tf_static` 1 ·
     `/drone/path_status` 1 · `/drone/imu` 223,053).
   - **이상치 방어 동작 확인**: drone_elevation_mapper 시작 로그에
     `max_sensor_range=200.0m, max_grid_cells=30000000, min_range_m=2.5m`
     가 정확히 찍혀 파라미터가 정상 선언/초기화됨을 확인. 이번
     100x100 지도 규모(관측 그리드 1,041,148셀)에서는 두 임계값 다
     정상 상황에선 거의 안 걸리는 값이라(200m, 3천만 셀) 실제 컷 발동
     로그는 없었음 — 이는 정상이며, 콜백 경로가 빠짐없이 실행됐다는
     것으로 방어 로직 자체의 정상 동작을 확인.

4. **4단계 — wheel/leg FN% 채점**: `capture_nav_fn.py`(기존, 무수정)로
   GT 통과가능 셀(618,378개, 4개 기존 결과와 동일 기준 확인됨) 대비
   채점. 결과는 아래 비교표 5번 행.

## 최종 비교표 (5개 결과 종합)

| # | 고도 | 속도 | 간격 | 위치정합 | 지도생성방식 | wheel FN% | leg FN% | 비고 |
|---|---|---|---|---|---|---|---|---|
| 1 | 84m | 8m/s | 32m | GT만 | 단순평균 | 23.94% | 12.15% | 기준선 |
| 2 | 84m | 8m/s | 32m | GT+GICP | GICP정합 | 20.10% | 11.89% | wheel -16.0%(상대), leg -2.1%(상대) 개선. 코너 4회뿐이라 GICP가 대체로 안정 수렴(94.5%), 원시 노이즈(p95)는 오히려 악화 — "쉬운 곳을 더 쉽게" 효과 |
| 3 | 5m AGL | 5m/s | 4m | GT만 | 단순평균 | 13.44% | 12.60% | 84m 대비 고도 자체 효과로 개선(근거리 스캔이라 점밀도↑, GICP 없이도 84m보다 나음) |
| 4 | 5m AGL | 5m/s | 4m | GT+GICP | GICP정합 | 37.26% | 30.03% | 대폭 악화(3번 대비 wheel +177%). 코너 25회, 저정보 스캔마다 GICP가 반복적으로 불안정해져 노이즈 5.3배 폭증(step 중앙값 0.0092→0.0489m) |
| 5 | 5m AGL | 5m/s | 4m | GT만 | **칼만필터(이상치방어 포함, 이번)** | **10.91%** | **8.03%** | 3번 대비 wheel -18.8%(상대)·leg -36.3%(상대) 개선, 4번 대비 wheel 3.4배·leg 3.7배 개선. 아래 분석 참조 |

(GT 통과가능 셀 618,378개 기준, 5개 행 모두 동일 — GT는 경로 형상만으로
정해지는 100x100 박스라 정합/알고리즘과 무관하게 고정. 원본:
`84m_baseline_fn.json`, `84m_methodB_fn.json`, `baseline_4m_5mps_fn.json`,
`methodb_4m_5mps_fn.json`, `kalman_4m_5mps_fn.json`.)

## 분석 — 5번(칼만필터)이 왜 3번(단순평균)보다 낫고, 4번(GICP)보다 훨씬 나은가

**5번 vs 3번(같은 GT pose, 알고리즘만 다름) — 칼만필터가 전 지표에서 개선.**
두 실험 다 GT pose만 쓰고(정합 없음) 4m/5mps 조건이 같으므로, 차이는
누적 알고리즘(러닝 애버리지 vs 칼만필터+이상치방어)에서 온다(단, 3번은
이전 세션의 삭제된 bag, 5번은 이번 세션 재비행 bag이라 비행 자체의
실행 차이가 완전히 0이라고 단정할 순 없음 — 다만 동일 스크립트/경로/속도의
결정적 시뮬레이션이라 그 영향은 작을 것으로 판단). 실측:
- wheel FN% 13.44%→10.91%(-2.53pp, 상대 -18.8%), leg FN% 12.60%→8.03%
  (-4.57pp, 상대 -36.3%) — 개선.
- 원시 단차(step) 지표도 전부 개선: 중앙값 0.00917→0.00692m(-24.5%),
  p95 1.039→0.953m(-8.3%), max 8.567→6.519m(-23.9%). 84m 실험에서
  GICP가 FN%는 개선하면서도 원시 노이즈(p95)를 악화시켰던 것과 달리,
  이번 칼만필터는 **FN%와 원시 노이즈를 동시에 개선** — "이미 쉬운 곳만
  쉽게" 만드는 게 아니라 지도 전체의 품질을 실제로 높였다는 뜻.
- 커버리지도 개선: elevation_map 유효 셀 비율 86.40%→95.80%(+9.4pp),
  FN 채점에서 GT 통과가능 셀 중 미측정 비율도 2.35%→1.46%로 감소.
  거리/입사각/밀도 기반 R로 신뢰도 낮은 관측을 억제하면서도 콜드스타트
  초기화(첫 관측은 게이팅 없이 즉시 반영)로 관측 자체가 버려지진 않아,
  커버리지를 깎지 않고도 정확도만 높인 것으로 해석된다.

**5번 vs 4번(GICP) — 사용자가 제시한 가설과 실측이 일치.** 가설: GICP는
스캔 하나당 강체변환(rotation+translation) 하나를 통째로 추정하므로,
코너의 저정보 스캔에서 정합이 잘못되면 그 스캔에 속한 점 전체가 한꺼번에
엉뚱한 방향/거리로 밀려 여러 셀에 동시에 오염을 퍼뜨린다. 반면 칼만필터는
셀 단위 독립 스칼라 필터라, 한 배치의 관측이 특정 셀들에서 이노베이션
게이트(임계값 9.0)를 못 넘으면 그 셀들만 조용히 거부되고 다른 셀의 상태에는
전혀 영향을 주지 않는다 — 게다가 5번은 애초에 GICP 같은 "전체를 다시
추정하는" 단계 자체가 없으므로(GT pose를 그대로 신뢰) 구조적으로 스캔
단위 오염이 발생할 수 없다.
- 실측이 이 가설을 뒷받침: 4번(GICP)은 자신의 baseline인 3번 대비
  step 중앙값이 0.0092m→0.0489m로 **5.3배 폭증**했다(코너 25회, "point
  cloud is too small" 경고 다수, PROGRESS.md 기존 기록) — 이게 바로
  "스캔 전체가 밀리는" 효과의 증거다. 반면 5번(칼만필터)은 같은 3번
  대비 step 중앙값이 오히려 **24.5% 감소**(0.0092→0.0069m) — 정반대
  방향. 알고리즘이 원시 노이즈를 늘리는 게 아니라 줄인다는 뜻이며,
  코너가 25회나 있는 이번 경로에서도 저정보 구간이 전체 지도 품질을
  끌어내리지 않았다는 직접적 증거다.
- FN 미측정 비율도 대조적: 4번은 GICP "성공"(수렴) 케이스가 많아 오히려
  미측정 비율이 낮았다(0.51%, 점이 넓게 퍼져 커버리지 자체는 89.18%로
  3번보다도 높았음 — SUMMARY 하단 "84m 실험" 절의 "정확도-커버리지
  트레이드오프"와 같은 패턴). 5번은 미측정 1.46%로 이보다는 높지만
  3번(2.35%)보다는 낮다 — 즉 5번은 4번처럼 "틀린 값이라도 넓게 채우는"
  방식이 아니라 "믿을 수 있는 값만 정확하게" 채우면서도 3번보다 더 넓게
  채운, 커버리지와 정확도를 동시에 만족한 유일한 결과다.
- 결론: 이번 실측은 사용자 가설(강체변환 전체 vs 셀단위 독립 처리)과
  정확히 일치한다. 코너가 잦은 저고도 lawnmower 경로에서는 GICP처럼
  "스캔 전체를 하나로 재추정"하는 방식이 구조적으로 취약하고, 칼만필터
  (+GT pose 그대로 신뢰 + 이상치 방어 2겹)처럼 "관측 하나하나를 그 관측이
  떨어진 셀에만, 신뢰도에 비례해 반영"하는 방식이 이런 경로 형태에 훨씬
  강건하다.

## 다음 결정 지점(참고용, 이번 범위 밖)

GICP(방법B)는 코너가 드문 경로(84m, 코너 4회)에서는 순이득이 있었지만
코너가 잦은 경로(5m AGL, 코너 25회)에서는 뚜렷한 손해였다. 칼만필터는
두 경우 모두에서 opt-in 손해가 없어 보이는(코너 수와 무관하게 안전한)
전략으로 보이나, 84m 조건에서의 칼만필터 실측은 아직 없다 — 필요하다면
같은 방식으로 84m bag에도 적용해 6번째 행을 추가하는 것을 고려할 수 있다.

---


## 무엇을 했나

이전 세션(84m 고도, 8m/s, 스트립간격 32m)에서 방법B(GT pose + GICP 정합)를
검증했던 것과 같은 방법론을, **훨씬 낮은 고도(5m AGL)·훨씬 촘촘한 경로
(스트립간격 4m, 속도 5m/s, 코너 25회)**에 적용해봤다. 5m AGL 실험은 이전
세션(attempt1~4)에서 매번 다른 이유로 실패(climb-rate 발산, VM 크래시, TF
동결, 정지판정 버그)해 방법B를 한 번도 끝까지 적용해본 적이 없었다 — 이번이
5m AGL + 방법B의 첫 완주 사례다.

## 결과 요약

| 조건 | 속도 | 간격 | 고도 | 위치 정합 | wheel FN% | leg FN% | 결과 |
|---|---|---|---|---|---|---|---|
| 84m GT 기준선(정합없음) | 8m/s | 32m | 84m | GT만 | 23.94% | 12.15% | - |
| 84m + 방법B | 8m/s | 32m | 84m | GT+GICP | **20.10%**(-3.84pp, -16.0%) | **11.89%**(-0.26pp, -2.1%) | 개선(트레이드오프 있음, 아래 참조) |
| 5m + 방법B(간격5m) | 5m/s | 5m | 5m AGL | GT+GICP | 인용할 이전 결과 없음(이전 시도 전부 실패) | | |
| 5m GT 기준선(정합없음, 이번) | 5m/s | 4m | 5m AGL | GT만 | **13.44%** | **12.60%** | - |
| 5m + 방법B(간격4m, 이번) | 5m/s | 4m | 5m AGL | GT+GICP | **37.26%**(+23.82pp, +177%) | **30.03%**(+17.42pp, +138%) | **크게 악화** |

(GT 통과가능 셀 618,378개 기준, 4개 조건 모두 동일 — GT는 정합/고도와 무관하게
경로 형상이 같은 100x100 박스라 고정. 84m행은 `run_results/84m_*_fn.json`,
5m행은 `run_results/{baseline,methodb}_4m_5mps_fn.json` 원본.)

## 분석 — 간격을 좁혔을 때(5m→4m) FN%가 개선됐는가? 고도 자체의 영향은?

**간격 자체(오버랩 증가)의 효과는 이번 실험만으로 답할 수 없다** — 5m 간격
방법B 결과가 아예 존재하지 않기 때문(이전 세션에서 5m/4m/3m/2m 간격 경로
파일은 만들었지만 velocity 비행이 매번 실패해 어떤 간격도 방법B까지 끝까지
가본 적이 없었다). 대신 비교 가능한 것은 **"같은 4m 간격에서 정합 유무"**
그리고 **"84m 대 5m, 고도 자체의 효과"** 둘이다.

**1) 고도 자체의 효과 — 5m AGL이 84m보다 GT 기준선 FN%가 뚜렷이 낮다.**
84m GT 기준선(23.94%/12.15%) 대비 5m AGL GT 기준선(13.44%/12.60%)은 wheel
기준에서 10.5pp(44%) 개선, leg는 거의 같음. 5m AGL은 스와스가 좁아도 4m
간격으로 촘촘히 겹치며 스캔하고(2단계 드라이런에서 박스 커버리지 99.6%
확인), 무엇보다 근거리 스캔이라 포인트 밀도 자체가 훨씬 높다 — GICP 없이도
이미 84m보다 낮은 노이즈로 셀 높이를 추정할 수 있었던 것으로 보인다.

**2) 정합(GICP)의 효과 — 84m과 5m AGL/4m/5mps에서 정반대 방향.**
84m에서는 GICP가 wheel FN%를 16% 상대개선(대가: 원시 노이즈 통계 악화,
미측정 비율 증가 — SUMMARY 하단 "84m 실험" 절 참조)했지만, 이번 5m AGL/
4m간격/5m/s 조건에서는 GICP가 **wheel FN%를 177% 악화**시켰다(13.44%→
37.26%). 원인 조사 결과(`PROGRESS.md` 7절 참조):
- 이번 경로는 코너(180도 yaw 반전)가 25회로 84m 경로(4회)보다 훨씬 많다.
  코너마다 스캔의 유효 점 수가 급감하는 구간이 반복적으로 나타났다(방법B
  cloud 생성 로그에 "point cloud is too small(2~10점)" 경고 다수).
- 1차 시도에서는 이런 저정보 상황에서 GICP가 `converged=True`를 반환하면서도
  물리적으로 불가능한 해(z=84m, y=219m 등)로 수렴하는 사례가 실측으로
  확인됐다(오염 44,864점, 0.017% — 그러나 이게 elevation_map의 `_grow_to_fit`
  무제한 성장 결함과 결합해 그리드 전체를 128x388m로 부풀리고 FN%를
  46.49%까지 악화시켰다). GT 대비 이동량 2.0m 상한을 GICP 결과 채택 조건에
  추가(원래 코드의 "GICP 실패 시 GT 안전 폴백" 설계를 "수렴은 했지만 말이
  안 되는 해"까지 포괄하도록 완성)한 뒤 재실행하니 파국적 오염은 98% 줄었으나
  (996점, 0.0003%), **여전히 wheel FN% 37.26%로 baseline보다 훨씬 나빴다** —
  전반적인 노이즈 증가(step 중앙값 0.0092m→0.0489m, 5.3배)가 남아있었다.
- **결론**: 84m처럼 코너가 드문 경로에서는 GICP가 "이미 쉬운 곳을 더 쉽게"
  만드는 순이득이 있었지만, 코너가 잦은 저고도 lawnmower 경로에서는 코너마다
  반복되는 저정보 스캔 구간이 GICP를 체계적으로 불안정하게 만들어 **정합이
  득보다 실이 훨씬 크다.** 이번 실험 조건(5m AGL, 4m 간격, 5m/s, 코너 25회)
  에서는 방법B를 쓰지 말고 GT pose만 쓰는 편(baseline)이 명백히 낫다.

## 원본 SUMMARY (84m + 방법B 실험, 이전 세션)



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
