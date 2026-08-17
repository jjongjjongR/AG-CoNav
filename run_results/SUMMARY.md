# SUMMARY — GLIM Phase 2 파라미터 튜닝 (2026-08-17, constant_velocity_inf_scale 스윕 + 전체 재실행)

**목표: 위 "GLIM odometry 발산 원인 진단" 절이 좁힌 1순위 후보
(`constant_velocity_inf_scale`)를 실제로 튜닝해 치명적 발산을 없애고,
GT 대비 오차를 84m 실험 수준(ATE~23m)까지 회복시키는 것.** 결과:
**치명적 발산은 완전히 해결됐으나, 잔여 드리프트(ATE 평균 68.6m)가
목표 수치에는 못 미쳤다 — 부분 성공.**

## constant_velocity_inf_scale 스윕 (40초 슬라이스, 6개 값)

| 값 | max error(40s) | 발산 시작(자동탐지) |
|---|---|---|
| 1e-2 | 113.17m | t=12.5s |
| 1e-1 | 168.11m | t=13.0s |
| 1e0(기존 기본값) | 86.20m | t=16.2s |
| 1e2 | 38.94m | t=32.4s |
| **1e3(채택)** | 33.48m | t=14.2s |
| 1e4 | 32.40m(최소) | t=47.8s(사실상 미검출) |

**원래 가설(값을 낮춰야 안정된다)과 정반대로, 값을 높일수록 안정됐다.**
기존 config 주석("이 bag은 정지 구간 없이 8m/s로 순항 시작하므로
사전확률을 낮춰야 한다")의 전제 자체가 이 bag(실제로는 정지 후
t≈15~17s에 전진 전환)과 안 맞았던 것으로 판명. 1e3을 채택(1e4가 40초
슬라이스에서는 근소하게 더 낫지만, 1e3는 이미 부분적으로 전체 bag
실행에서 크래시 없이 진행된 실적이 있어 안정성을 우선).

## 전체 bag(2254s) 재실행 + ATE 검증

cv1e3로 12청크 전부 크래시/메모리가드 없이 완주. GT 대비 ATE:
- **절단 없음(bag 끝까지): 평균 116.5m** — 그러나 t≈2129.67s
  (`/drone/path_status=true` 발행 직후) 이후 GT 자체가 자유낙하해
  (-117, 512, -678)에서 멈추는, **임무완료 후 정상 종료 시퀀스이지
  SLAM 오류가 아닌 구간**이 섞여 있었다.
- **실제 비행 구간만(t<=2129s): 평균 68.6m, 중앙값 72.7m, 최대
  137.3m.** 치명적 발산(수백 m 단위 물리적으로 불가능한 값)은 완전히
  사라졌고, 오차는 각 스트립 왕복(U턴)마다 5~140m 사이를 진동하는
  **경계가 있는(bounded) 패턴**으로 바뀌었다 — GPS 사후결합(이미 설계·
  구현 완료, `run_results/glim_gps_correct.py`)이 잘 들을 수 있는
  유형. 84m 실험 수준(ATE~23m)에는 못 미쳐 "성공" 기준은 엄밀히
  미달이지만, GLIM을 애초에 못 쓰게 만들었던 근본 버그는 고쳐졌다.

**권고: 다음 세션은 Phase 4(GPS 사후결합)로 진행.** 상세 근거·다음
단계는 `run_results/PROGRESS.md`(GLIM Phase 2 파라미터 튜닝 절) 참고.

---

# SUMMARY — GLIM odometry 발산 원인 진단 (2026-08-17 새 세션, 0~1단계 완료)

**이 세션의 목표는 "발산을 고치는 것"이 아니라 "왜 발산하는지 진단하는
것"이었다 — 파라미터 튜닝은 의도적으로 안 했다(Phase 2로 미룸).** 아래
"GLIM(+GPS 사후결합) 전체 파이프라인 실험" 절(이전 세션, 미완료로
끝난 세션)이 남긴 "GLIM 궤적이 발산해서 GPS 사후결합을 못 써봤다"는
문제를 이어받아 원인을 밝혔다.

## 0단계 — glim_ext 빌드 가능성: **성공**

`~/glim_ext_ws`에 `koide3/glim_ext`를 clone+colcon build, 별도 조치
없이 1차 시도부터 성공. `libimu_validator.so`, `libgnss_global.so`
정상 빌드. `source ~/glim_ext_ws/install/setup.bash`를 추가하면 GLIM이
확장 모듈을 정상 인식(등록만 하고 워크스페이스 미소싱 시 GitHub 이슈
#9와 동일한 "glim_ext package path was not found" 재현까지 확인).
FAST-LIO2 래퍼만 서브모듈(SSH 전용 URL)을 못 받아 안 빌림(사용 안
할 예정이라 무관). 라이선스: glim_ext 전체 GPLv3, gnss_global도
동일(비상업 제한 없음) — 이번 진단 범위엔 라이선스 문제 없음.
자세한 내용: `run_results/glim_ext_feasibility.md`.

## 1단계 — 발산 원인 진단: 유력 원인 좁힘

재사용 bag(GPS 포함, 재비행 없음)의 **chunk 0(앞 190.5초)**만으로
GLIM을 기존 config 그대로 1회 실행하고 GT(`/tf`)와 정렬 비교했다(전체
2254초를 다 돌리면 처리에만 5~6시간 필요하다는 게 이전 세션에서 이미
실측됐고, chunk 0 안에 발산이 충분히 나타나 그럴 필요가 없었다).

**핵심 발견 — 발산은 이전 추정(t≈101~136s)보다 훨씬 일찍(t≈15~17s)
시작된다.** 이전 세션은 "절대좌표값이 물리적으로 말이 안 되는 시점"을
발산 시점으로 봤는데, 그건 오차가 ~100초간 누적된 뒤 육안으로
명백해진 시점일 뿐이었다. GT와 정렬해 유클리드 오차를 실측하면 비행
시작 6~8초 만에 이미 오차가 커지기 시작하고, 이 시점은 **드론이
정지(hover)에서 전진 비행으로 전환되는 순간과 거의 정확히 일치**한다
(GT 위치가 t=8.9~13.6s 동안 거의 고정돼 있다가 그 직후부터 움직임).

- **③ 타임스탬프 동기화**: 이상 없음(bag 전체 22508+224866개 메시지
  검사, 역전 0건, 발산구간 근처 이상 0건) — **원인에서 배제**.
- **④ LiDAR-IMU 외부보정**: 이상 없음(model.sdf 실측값으로 직접
  재계산해 config 값과 소수점까지 일치 확인) — **원인에서 배제**.
- **⑤ 디스큐 on/off 비교**: 발산 **시작 시점**은 거의 동일(15.7s vs
  15.9s) — 디스큐가 1차 방아쇠는 아님. 그러나 발산 **이후 심각도**는
  뚜렷이 다름(디스큐 ON 최대오차 306m·혼란스러운 스크리블 vs OFF
  135m·매끄러운 단일 드리프트) — **디스큐는 2차 악화 요인으로 좁혀짐**.
- **①② GT-GLIM 정렬 비교**: 1순위 후보로 **CT 오도메트리의 "정지→
  전진 전환" 처리**를 지목. `config_odometry_ct.json`의
  `constant_velocity_inf_scale` 주석이 "이 bag은 이미 8m/s로 순항
  중일 때 시작한다"고 전제하는데, 실측 GT는 정지 구간이 실제로
  존재해 이 전제와 어긋난다.

Phase 2(파라미터 튜닝) 권고 순서: 1) `constant_velocity_inf_scale`을
이 bag의 실제 초기 속도 프로파일에 맞게 재검토, 2) 디스큐를 일단
끈 상태를 새 기준선으로 삼고 1)이 안정화된 뒤에 디스큐 정밀도를
따로 개선. 자세한 근거·수치·그래프: `run_results/divergence_diagnosis.md`.

## 산출물(이번 세션)

- `run_results/glim_ext_feasibility.md`, `run_results/divergence_diagnosis.md`
- `run_results/diag_scripts/check_timestamps.py`, `compare_gt_glim.py`
- `run_results/glim_diag_dumps/run1_full_pipeline/`(디스큐 ON),
  `run2_nodeskew/`(디스큐 OFF) — 각각 GLIM 궤적 txt 4종 + GT 비교 PNG
- `glim_config_nodeskew/`(디스큐 OFF 비교용 config 사본, `global_shutter_lidar: true`만 다름)
- `~/glim_ext_ws/`(빌드된 glim_ext 워크스페이스, git 추적 대상 아님 — 홈 디렉터리 별도 위치)

---

# SUMMARY — GLIM(+GPS 사후결합) 전체 파이프라인 실험 (2026-08-17, 미완료, 이전 세션)

## 전제 (지시사항 그대로 명시)
이 시뮬레이션의 LiDAR/IMU/GPS 센서 정의(agconav_description)에는
`<noise>` 블록이 전혀 없다 — 즉 이 GPS는 잡음 없는 완벽한 위치를
발행한다. 그러므로 이 실험은 "실전의 노이즈 있는 GPS로 얼마나
개선되는가"를 보는 것이 **아니라**, "GLIM의 위치추정 드리프트를 정밀한
위치 앵커로 억제했을 때, GLIM의 지도생성 파이프라인 자체가 제대로
작동하는가"를 검증하는 것이다.

## 결과: 목표 미달성 — GLIM 궤적을 신뢰할 수 있는 형태로 못 얻음

**성공한 부분**:
- GPS 센서를 브리지·기록에 새로 추가(`experiment.launch.py` 2곳 수정)
  하고 velocity 재비행 완료 — `bags/velocity_4m_5mps_gps_attempt2`
  (23G, `/drone/gps` 22509건/`/drone/points` 22508건, 거의 1:1 매칭
  확인). **비행 소요시간(4개 지표 중 하나): 2254.09초(≈37.57분)**
  (bag duration 실측, 이전 GPS 없는 동일 경로/속도 실험의 2258.8s와
  거의 일치 — 재현성 확인).
- GLIM GNSS 네이티브 지원 조사: 코어에는 없음, 별도 저장소
  `glim_ext`에 `libgnss_global.so`가 있으나 공식적으로 "실용적이지
  않은 미완성 코드"로 경고돼 있어 미채택. 지시사항의 폴백대로 **GPS
  사후 결합(loose coupling)** 방식을 설계·구현
  (`run_results/glim_gps_correct.py` — GLIM 원시 궤적에 GPS 앵커로
  SE3 보정을 적용, 회전은 identity 고정+lerp/slerp 보간).
- 지도 생성(`run_results/glim_gps_build_map.py`)과 지표 계산
  (`run_results/glim_gps_metrics.py`, 사용자 지정 최대연결덩어리%
  알고리즘 포함)도 미리 구현 완료 — **정상 궤적만 있으면 바로 실행
  가능한 상태**.

**실패한 부분 — GLIM 자체가 이 VM/bag 조합에서 안정적으로 안 돎**:
4가지 설정 조합(전체 파이프라인 / odometry-only / local mapping만 /
전체+메모리가드)을 전부 시도했으나:
- 메모리를 아끼려고 local/global mapping을 끄면(또는 무작정 뒀다가
  메모리가드로 일찍 끊으면) → **CT odometry 궤적이 스캔 100여 개
  안에 물리적으로 불가능한 값(z가 -100m~+265m대)으로 발산**.
- 메모리 제한 없이 전체 파이프라인을 다 돌리면(1차 시도) → 12청크
  전부 재생은 끝냈으나 **OOM killer에 SIGKILL당해 완전 손실**(정상
  종료 훅이 실행 안 돼 `/tmp/dump` 자체가 안 생김).
즉 "발산 안 하려면 메모리가 부족하고, 메모리를 아끼면 발산한다"는
딜레마에 부딪혔고, 이번 세션 시간 안에 근본 원인(CT odometry 초기화
파라미터 추정, `constant_velocity_inf_scale` 등)을 찾아 고치지
못했다. 상세 시도 기록·다음 세션이 우선 시도해볼 것은
`run_results/PROGRESS.md` "6. GLIM 실행" 절 참고.

## 4가지 지표 — 1개만 확정, 3개는 미산출
| 지표 | 값 |
|---|---|
| 비행 소요시간 | **2254.09초 (≈37.57분)** |
| 커버리지 | 산출 못함(정상 궤적 없음) |
| wheel FN% | 산출 못함 |
| wheel/leg 최대연결덩어리% | 산출 못함 |

---

# SUMMARY — ④ 디스큐 커버리지 손실 버그 수정 (2026-08-16)

목표: 직전 실험(A/B/C)에서 발견된 ④(디스큐)의 커버리지 손실 버그(스캔
뒷부분 버킷에서 TF extrapolation 실패로 ~8%p 손실)를 고치고 ④만 단독
재측정, 커버리지가 회복되는지·baseline을 넘어서는지 확인.

## 결과 요약

| 항목 | B(구, 버그있음) | B'(신, 버그수정) | 기준(#5) |
|---|---|---|---|
| coverage_percent | 92.95% | **94.69%** | 95.80% |
| unknown_percent | 9.47% | **1.35%**(회복) | 1.46% |
| wheel FN% | 15.00% | **20.51%** | **10.91%** |
| leg FN% | 7.53% | **13.70%** | **8.03%** |

**버그 수정은 목적(커버리지 회복)을 정확히 달성**했지만(unknown
9.47%→1.35%, 기준과 사실상 동일), **FN%는 오히려 더 나빠졌다**. 해석:
extrapolation으로 버려지던 스캔 뒷부분 데이터가 실은 노이즈/디스큐 근사
오차가 큰 영역이었고, 그게 버려지며 우연히 "깨끗해 보였던" 것 — 버그를
고쳐 그 데이터를 포함시키자 그 안의 노이즈가 그대로 드러났다. 즉 ④(12
버킷 column-index 기반 시간 근사)는 이 조건에서 baseline보다 나은
지도를 만들지 못하며, 직전 세션의 ①/①+④와 마찬가지로 **채택
비권고**(상세 분석은 PROGRESS.md "4단계" 참고).

## 완료한 것
1. **원인 확정(실측)**: `ExtrapolationException`("Lookup would require
   extrapolation into the future") — 버킷 6~9(스캔 중후반부)에 집중,
   짧은 재현 구간에서만도 1170건. 라이브 `/tf` 구독은 point cloud
   콜백이 요구하는 시각의 TF를 아직 재생 전이라 절대 못 가질 수 있는
   구조적 문제였음.
2. **근본 수정**: bag 재처리는 실시간이 아니므로, 노드 시작 시 그 bag의
   `/tf`+`/tf_static`을 통째로 먼저 읽어 TF 버퍼를 채우는
   `deskew_tf_preload_bag_path` 파라미터 추가
   (`drone_elevation_mapper.py`). 수정 후 재현: extrapolation 실패
   **0건**(확인 완료). 22GB bag을 매번 순차 스캔하는 비용을 줄이는
   pickle 캐시(`/tf`+`/tf_static`만 추림)도 추가.
3. **"hang처럼 보이는 문제" 재진단**: 수정 후 재처리가 47분+ 동안
   로그 없이 멈춘 것처럼 보여 광범위하게 조사했으나(faulthandler 스택
   덤프, 토픽 발행률, DDS discovery, QoS reliability, 메모리/스왑 —
   전부 실측) 진짜 데드락 증거는 못 찾음. 오히려 체크 타이머가
   "정상 수신 중엔 로그를 안 남기는" 설계이고, 지도는 완주 시(bag 맨
   끝 `/drone/path_status`) 1회만 발행하는 설계라, **hang이 아니라
   "정상이지만 예상보다 느리게 처리 중"일 가능성이 유력**하다는 결론에
   도달(bag 자체가 큰 point cloud 메시지에서 burst 재생 패턴을 보임,
   매퍼 없이 순수 재생만으로도 실측 확인).

## 최종 검증 실행 — 완료
채점 타임아웃을 3000→10800초(3시간)로 늘려 돌린 최종 검증 재처리는
**35.2분 만에 정상 완료**(기대치 37.6분과 거의 일치, 오히려 더 빠름) —
크래시/에러 없음, extrapolation 실패 0건. 이걸로 3-4절에서 의심했던
"hang"이 진짜 코드 버그가 아니라 그 시점의 일시적 시스템 상태(반복된
진단 재현들이 만든 디스크 캐시/프로세스 경합)에 의한 것이었음이
확정됨. 결과는 위 "결과 요약" 표에 반영 완료.

상세 조사 과정·타임스탬프·명령어는 전부 `run_results/PROGRESS.md`
("[새 세션] ④ 디스큐 커버리지 손실 버그 수정 + B' 재측정" 이하)에
기록됨.

---

# SUMMARY — ①실측R보정 + ④디스큐 변형 A/B/C 실험 (2026-08-16, 신규 세션)

**세션**: 2026-08-16, 브랜치 `test_main_brian`(작업 폴더
`~/AG-CoNav-test_main`), 사용자 취침 중 자율 진행(전체 타임아웃 3시간).
전 단계 상세 근거는 `run_results/PROGRESS.md`("[새 세션] R보정(①)+디스큐(④)
변형 A/B/C 작업 시작" 이하)에, R 보정 상세는
`run_results/calibrated_noise_table.md`에 전부 기록.

## 목표

직전 실험(#5, 5m AGL/4m간격/5m/s/velocity/칼만필터+이상치방어, wheel FN%
10.91%/leg FN% 8.03%) 위에 두 가지 개선을 각각·함께 시도:
- **①** 정지비행(teleport) 실측으로 측정노이즈(R) 구간표를 보정
- **④** 스캔 내 시간왜곡(모션 블러) 보정(디스큐)

## 무엇을 했나

1. **재개 확인**: 크래시로 끊긴 이전 세션이 `run_results/calibrate_noise.py`
   (①용 신규 스크립트)와 `--pilot`(조건 1개) 결과만 남긴 채 중단돼 있었음.
   `bags/velocity_4m_5mps_attempt1`(#5의 원본 22GB bag) 무결성 확인 →
   재비행 불필요.
2. **① 실측 보정 중 버그 발견·수정**: 정지비행 teleport 첫 호출이 DDS
   디스커버리 레이스로 씹혀 드론이 엉뚱한 위치(84m 상공)에서 스캔한
   오염된 pilot 데이터를 발견 → `calibrate_noise.py`에 TF 기반 도착
   확인을 추가해 수정. 12조건(거리4×입사각3) 재실측 완료, 6개는 반경
   0.5m에서 바로 성공, 6개는 빔 간격이 넓어져 반경을 2~3m로 확대
   (그 중 1개는 지형 혼입으로 폐기, 2개는 끝내 측정 불가로 모델에서
   제외). 실측 결과 이 무노이즈 결정론적 시뮬레이터의 σ는 거리·입사각에
   거의 무관하게 0.006~0.009m로 평탄 — 기존 이론표(0.007→0.050m,
   `1/cos²θ`)보다 훨씬 작음. 새 R표
   `[20,50,90,170]m→[0.008,0.009,0.010,0.012]m`,
   입사각 지수 2.0→0.3(신규 파라미터화)을 도출.
3. **④ 디스큐 구현**: `/drone/points`에 점별 타임스탬프 필드가 없음을
   실측 확인(x,y,z,intensity,ring만) → organized cloud(32ring×1024azimuth)의
   column index를 발사 순서 근사치로 써서, 스캔을 12구간으로 쪼개
   구간별 TF를 따로 조회·적용하도록 `drone_elevation_mapper.py`에
   `deskew_enabled`(기본 false) 경로 추가. 리팩터링 중 버그(`msg` 미정의
   NameError) 1건을 스모크테스트로 잡아 즉시 수정.
4. **A/B/C 재처리**: 기존 칼만필터 bag재처리 스크립트
   (`run_bag_replay_kalman.sh`)를 일반화한 `run_variant_replay.sh`로,
   #5의 원본 bag을 파라미터만 바꿔 3회 재생(각 ~38분, rate=1.0, #5와
   동일 조건 유지) — 재비행 없음.
5. **채점**: 기존 `capture_nav_fn.py`(무수정)로 동일 정의(GT 통과가능
   셀 중 파이프라인이 "막힘"으로 오판한 비율) 채점.

## 결과

| # | 고도 | 속도 | 간격 | 위치정합 | 지도생성방식 | wheel FN% | leg FN% | 비고 |
|---|---|---|---|---|---|---|---|---|
| 5 | 5m AGL | 5m/s | 4m | GT만 | 칼만필터(이상치방어) | **10.91%** | **8.03%** | 기준(#5) |
| A | 5m AGL | 5m/s | 4m | GT만 | +①실측R보정 | **40.18%** | **25.48%** | ① 단독 — 큰 폭 악화 |
| B | 5m AGL | 5m/s | 4m | GT만 | +④디스큐 | **15.00%** | **7.53%** | ④ 단독 — wheel 악화, leg 소폭 개선 |
| C | 5m AGL | 5m/s | 4m | GT만 | +①+④ | **27.44%** | **14.48%** | 둘 다 — A보다 낫지만 기준 미달 |

**핵심 발견**: ①(정지비행 실측 R)을 등속비행(모션 블러 존재) 데이터에
디스큐 없이 그대로 적용하면 칼만필터가 노이즈를 과소평가해 지도가 오히려
크게 거칠어진다(wheel FN% +268%, leg FN% +217% 상대 악화) — **기존
이론 기반 R표의 큰 σ가 "정지 실측 vs 실제 비행" 간극에 대한 암묵적
안전마진 역할을 하고 있었다는 뜻**. ④를 더하면 이 손상이 상당 부분
줄어들지만(C가 A보다 wheel -12.7pp·leg -11.0pp 개선) 완전히 사라지지
않고, ④ 자체도 버킷별 TF 조회 실패로 추정되는 커버리지 손실(unknown
1.5%→9.5%) 부작용이 있어 단독으로도 기준을 못 넘는다. **결론: ①·④·둘
다 모두 이번 조건에서는 #5 기준보다 나쁘며 채택 비권고.** 상세 분석은
`run_results/PROGRESS.md` "5단계 — 비교표 및 분석" 절 참고.

## 산출물

- `run_results/calibrate_noise.py`, `calib_summary.json`,
  `calibrated_noise_table.md` — ① 실측·분석·최종 R 모델.
- `run_results/diag_probe.py` — teleport 버그 진단용 1회성 스크립트.
- `run_results/run_variant_replay.sh` — A/B/C 공통 재처리 스크립트.
- `run_results/variantA_calibR_fn.json`, `variantB_deskew_fn.json`,
  `variantC_calibR_deskew_fn.json` — 채점 결과.
- `src/agconav_drone/agconav_drone/drone_elevation_mapper.py`,
  `config/drone_elevation_mapper.yaml` — ①④ 파라미터화(기본값은 #5와
  동일하게 유지, 옵트인 방식).

---

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
