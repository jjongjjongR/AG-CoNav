# CHANGELOG — kalman-filter-integration

이 브랜치는 `main`에서 분기했다. **실행 방법은 main과 완전히 동일하다**
(`colcon build --symlink-install` 후 `ros2 launch agconav_bringup
agconav_sim.launch.py`, 인자 없이). 월드/launch 기본 인자/속도/고도/경로/
물리엔진 설정은 전혀 건드리지 않았다 — 바뀐 것은 Module A(드론)와 Module
D(wheel/leg)의 지도 누적 알고리즘, Module E의 병합 알고리즘, 그리고 그
알고리즘이 쓰는 파라미터를 yaml에 새로 추가한 것뿐이다.

## 배경

이 저장소에는 이번 통합 이전에 이미 각 조각이 서로 다른 브랜치에서
개별적으로 검증돼 있었다:

- Module A의 셀별 칼만필터 + R(측정노이즈) 계산 + np.gradient 국소윈도우
  성능 최적화: `brian_test` 브랜치 (커밋 4172ea5, 043c0c2, 53d0a64).
- Module D의 셀별 칼만필터 + R 계산 + np.gradient 국소윈도우 최적화 +
  거리 기반 R을 계수 근사식에서 데이터시트 구간표로 교체: `main`에서
  직접 분기한 `kalman-filter-uncertainty-fusion` 브랜치 (커밋 0532135,
  4d12677, 5afd573).
- Module E의 분산 기반 wheel/leg 지도 선택: 같은
  `kalman-filter-uncertainty-fusion` 브랜치 (커밋 24ce33e).
- Module A/D 공통의 이상치 방어 2겹(max_sensor_range, max_grid_cells):
  `brian_test`(Module D, 커밋 90e0552)와 `test_main_brian`(Module A,
  커밋 de5610b, `brian_test`의 칼만필터를 100x100 축소 실험 월드로 이식하며
  Module D를 참고해 신규 구현).
- Module E의 저장 실패 시 상태 발행 + collector 타임아웃 감시:
  `brian_test` 브랜치 (커밋 fd43108).
- Module A의 건물경계 혼합모델(2가설 병렬 추적): `test_main_brian`
  브랜치에서 "변형 11"로 Module A의 5m AGL 실비행 bag에 대해 실험되어
  Phase 1 11개 변형 중 전체 최고 성적으로 채택됐다(커밋 e4b4612,
  2006c37 — `run_results/kf_variant_11_building_mixture.md` 문서 참고).
  다만 그 실험 세션은 "코드는 baseline으로 완전히 복원, 문서만 보고"로
  마무리되어 실제 코드 커밋은 어디에도 남아있지 않았다 — **이번이 그
  검증 결과를 실제 코드에 반영하는 첫 커밋**이다(문서에 기술된 정확한
  설계를 그대로 재구현했다).

이 브랜치는 이 조각들을 각 브랜치의 검증된 형태 그대로 `main` 위에
모으고, Module A에는 아직 코드로 존재하지 않던 건물경계 혼합모델을 문서
기반으로 처음 구현해 넣은 것이다. 새로 설계한 것은 없다.

## Module A (드론, `agconav_drone/drone_elevation_mapper.py`)

- **상태 표현**: `self._sum`/`self._count` 러닝애버리지를 셀별
  elevation/variance 칼만필터로 교체.
- **R(측정노이즈)**: 거리 구간표(`measurement_noise_max_distances`=
  [1.0, 20.0, 50.0, 100.0], `measurement_noise_sigmas`=[0.007, 0.010,
  0.020, 0.050], np.searchsorted 조회, 100m 초과는 마지막 구간 유지,
  시작 시 길이 일치·오름차순 검증) + 입사각(np.gradient로 국소 법선
  추정 → cos_theta, `incidence_cos_floor`=0.17로 하한, 이웃 정보 부족시
  cos_theta=1.0 폴백) + 점밀도(R_eff = R_point / n_points, R_point =
  R_distance / cos_theta**2).
- **np.gradient 국소 윈도우 (성능)**: 전체 배열이 아니라 이번 배치가
  건드린 영역 + 가장자리 1칸에만 np.gradient를 적용한다. **왜**: 이전에
  전체 배열에 적용했을 때 지도 규모가 커질수록(드론은 84m 상공에서 넓은
  면적을 스캔) 콜백 처리 시간이 계속 늘어나는 성능 버그를 실제로 겪었다
  (`brian_test` 커밋 53d0a64). np.gradient(edge_order=1 기본값)는 각 점의
  미분에 바로 이웃한 칸만 쓰므로, 윈도우가 그 이웃을 전부 포함하는 한
  결과는 수학적으로 완전히 동일하다.
- **Q=0**: 정적 지형 가정.
- **이노베이션 게이팅**: `innovation_gate_threshold`=9.0(카이제곱 자유도
  1, 약 3-시그마). 신규 셀(P=NaN)은 게이팅 없이 즉시 초기화.
- **칼만 갱신 (낭비 제거)**: 신규/기존 셀 판정에는 오직 P(분산) 배열만
  필요한데, 기존 구현은 elevation 값(x)도 신규 셀 위치를 포함한 배치
  전체에 대해 미리 읽어 뒀다가 나중에 버렸다. 이번에 신규/기존 판정 이후
  실제로 필요한 부분집합에서만 x를 직접 읽도록 바꿨다 — 결과값은
  수학적으로 완전히 동일함을 무작위 합성 데이터 200회 시행으로 검증했다
  (아래 "검증" 절 참고).
- **`_grow_to_fit` NaN 패딩**: np.pad의 패딩값을 0이 아니라 np.nan으로.
  **왜**: 0으로 패딩하면 그 셀의 분산이 0("완벽하게 확신한다"는 뜻)이
  되어 칼만게인 K=P/(P+R)이 영구히 0으로 고정되는 치명적 버그가 있다.
- **GridMap 출력**: `elevation_variance` 레이어 추가, elevation과 동일한
  축뒤집기+column-major 패킹. `basic_layers`는 `elevation`만 유지.
- **이상치 방어 2겹**: (1) `max_sensor_range`(기본 200.0m) — 센서 원점
  기준 이 거리를 넘는 점은 격자에 넣지 않고 버림(디버그 로그만). (2)
  `max_grid_cells`(기본 30,000,000) — `_grow_to_fit`이 패딩 전 예상 총
  셀 개수가 이 값을 넘으면 실제 확장을 하지 않고 배치를 버림(에러 로그,
  노드는 생존, 기존 누적은 보존).
- **건물경계 혼합모델** (신규 구현, 배경 절 참고): 셀별로 독립된 2개의
  칼만필터 가설을 병렬 추적 — 가설 A("지형"), 가설 B("지붕/장애물 경계"
  후보). 신규 셀은 A만 초기화(콜드스타트 동일). A만 있는 셀은 게이트
  통과 시 A 갱신, **실패 시(기본 칼만필터라면 버렸을 관측) B를 새로
  생성**해 관측을 보존한다. A/B가 둘 다 있는 셀은 정규화 이노베이션이
  더 작은 쪽에만 붙여 갱신하고, 둘 다 게이트를 통과 못하면 3번째 후보를
  만들지 않고 버린다(복잡도 2가설로 제한). 최종 발행은 분산이 더 작은
  가설을 채택(동률은 A 우선). `_grow_to_fit`도 4개 배열(A/B의
  elevation/variance)을 항상 같은 shape/origin으로 NaN 패딩한다.
  **왜**: 검증 문서(`kf_variant_11_building_mixture.md`)에 따르면 이
  설계가 하드 게이팅의 "이상치처럼 보이지만 사실은 다른 실제 지형/구조물"
  관측을 영구히 버리는 문제를 구조적으로 해결해, wheel FN% -6.30pp(상대
  59.0% 개선), leg FN% -3.61pp(상대 45.8% 개선), 커버리지도 개선,
  최대연결덩어리%도 wheel 27.58%→51.63%/leg 40.96%→54.51%로 크게
  개선되며 실험한 11개 변형 중 전체 최고 성적이었다. 게이팅 완화(16.0)나
  Huber 재가중과는 실험에서도 상호배타적으로 다뤄졌고(같은 지점을 다른
  방식으로 바꾸는 변형들) Phase 2 조합 실험에서 함께 적용해도 이득이
  거의 없었으므로(중복 효과) 이번 구현에도 섞지 않았다.
- **package.xml**: `rosbag2_py` 바로 다음 줄에
  `<exec_depend>rosbag2_storage_mcap</exec_depend>` 추가(mcap 저장에
  필요한 런타임 플러그인, 이전엔 선언 누락).
- **1-11 확인 결과**: `elevation_map_saver.py`는 main 기준으로 **이미**
  저장 실패 시 `Bool(False)`, 성공 시 `Bool(True)`를 발행하고 있어서
  건드리지 않았다.

## Module D (wheel/leg, `agconav_ground_mapping/ground_elevation_mapper.py`)

Module A와 완전히 동일한 설계(위 항목 전부)를 이식했다. 차이:

- R 구간표는 `leg_elevation_mapper.yaml`/`wheel_elevation_mapper.yaml`에
  각각 독립적으로 선언하되, 값은 둘 다 기존과 동일한 OS1-32 데이터시트
  기준 값을 그대로 유지했다. wheel이 실제로는 A300 lidar3d_0라는 다른
  센서를 쓴다는 건 이미 알려진 미해결 이슈지만, 지시대로 이번 작업
  범위에서 새로 측정하지 않았다.
- `ground_elevation_map_saver.py`: 저장 성공 시에는 이미 `Bool(True)`를
  발행하고 있었으나 실패 시에는 로그만 남기고 있었다 — `_save_elevation_map`
  이 실패를 반환하면 콜백에서 `Bool(False)`도 발행하도록 추가했다(성공
  로직은 그대로). **왜**: map_merge_collector가 wheel/leg 완료 상태를
  기다리는데, 저장이 실패하면 그 신호가 영영 안 와서 무한 대기하게 되기
  때문(아래 Module E의 merge_wait_timeout_sec와 짝을 이루는 수정).
- `package.xml`: 동일하게 `rosbag2_storage_mcap` exec_depend 추가.

## Module E (`agconav_map_fusion`)

- **`elevation_map_merger.py` + `grid_math.py`**: 고정 우선순위
  `MERGE_ORDER=('drone','leg','wheel')` 방식을 분산 기반으로 교체.
  wheel과 leg가 겹치는 셀은 `elevation_variance`가 더 낮은(더 확신
  있는) 쪽을 채택(동률은 wheel 우선, 결정론적 타이브레이크). 한쪽만
  관측한 셀은 그 값을 그대로 사용. 드론은 `elevation_variance` 비교에
  참여하지 않고, wheel/leg 둘 다 관측 못한 셀에만 최하위 폴백으로
  쓰인다(`MERGE_ORDER=('drone',)`로 축소). 전부 numpy 벡터화 연산
  (`_merge_ground_elevation`)이며 셀별 파이썬 반복문은 없다.
  `grid_math.py`에 `extract_elevation_variance` 헬퍼를 추가했다 —
  `elevation_variance` 레이어가 없는 지도(방어적으로, 레이어가 없는
  경우 일반)에는 None을 반환한다.
- **`map_merge_collector.py`**: `merge_wait_timeout_sec`(기본 30.0)
  파라미터 추가. 이 시간 안에 wheel·leg 둘 다 완료 상태가 안 오면 기존
  검증실패 처리와 동일한 경로(merge_error + merge_status=False)로
  발행한다. 타임아웃은 "노드 시작 또는 마지막으로 완료 상태가 바뀐
  시점"부터 재는 방식(상태가 실제로 바뀔 때만 기준 시각 리셋)이고,
  병합이 이미 트리거된 이후에는 더 이상 이 체크를 하지 않는다.
- **`package.xml`**: 동일하게 `rosbag2_storage_mcap` exec_depend 추가.

## yaml 반영

`drone_elevation_mapper.yaml`, `leg_elevation_mapper.yaml`,
`wheel_elevation_mapper.yaml`에 `measurement_noise_max_distances`/
`measurement_noise_sigmas`/`incidence_cos_floor`/
`innovation_gate_threshold`/`max_sensor_range`/`max_grid_cells`를,
`map_merge_collector.yaml`에 `merge_wait_timeout_sec`를 새로 추가했다.
`elevation_map_merger.py`는 이번 변경으로 새 파라미터가 늘지 않아
`elevation_map_merger.yaml`은 그대로 두었다. **기존에 있던 다른
파라미터의 값(예: `max_grid_cells: 30000000`)은 절대 바꾸지 않았다** —
`brian_test`에는 이 값을 500x500m 월드 규모에 맞춰 60,000,000으로 올린
별개의 수정이 있었지만, 이번 작업 범위(지도 누적/병합 *알고리즘*
교체)와 무관한 설정값 변경이라 main의 원래 값(30,000,000)을 그대로
유지했다.

## 검증

- `colcon build --symlink-install` — 전체 워크스페이스(15개 패키지)
  통과. 3개 수정 패키지(agconav_drone, agconav_ground_mapping,
  agconav_map_fusion) 단독 빌드도 별도로 통과 확인.
- **동치성 검증(1-5 낭비 제거 리팩토링)**: naive(배치 전체에서 elevation
  값을 미리 읽음) 구현과 optimized(신규/기존 판정 이후 필요한 부분집합
  에서만 읽음) 구현을 무작위 합성 데이터(이상치/게이트 실패/가설 B
  생성이 섞인 200회 시행, 각 시행 6개 배치)로 나란히 돌려
  `elevation_a`/`variance_a`/`elevation_b`/`variance_b` 4개 배열이
  전부 `np.array_equal(..., equal_nan=True)`로 완전히 동일함을 확인했다.
- **launch 짧은 검증: 미검증(환경 문제로 확인 못 함)**. `ros2 launch
  agconav_bringup agconav_sim.launch.py`(README에 문서화된 표준 명령,
  인자 없이)를 시도했으나, 이 로컬 환경에 설치된
  `config/clearpath_a300/platform/launch/platform-service.launch.py`
  (git 추적 대상, Clearpath 생성물)에 다른 사용자/클론 경로
  (`/home/lee/projects/AG-CoNav/...`)가 하드코딩돼 있어 xacro 처리
  단계에서 즉시 실패했다. 이 브랜치의 Kalman filter 변경과는 무관한
  기존 로컬 환경 문제이며(같은 문제가 main/brian_test에도 동일하게
  있을 것), 이번 작업 범위("지도 누적/병합 알고리즘 코드만 교체")를
  벗어나는 `config/clearpath_a300/` 수정 없이는 고칠 수 없어 손대지
  않았다. 사용자 지시에 따라 이 단계는 미검증으로 남기고 넘어간다 —
  colcon build 자체는 이미 성공했으므로 이 실패가 전체 작업을 막지는
  않는다. (`brian_test`에 이 정확한 파일을 고치는 uncommitted 작업이
  있던 것으로 보아, 사용자가 이미 알고 있는 별도 이슈로 보인다.)

## 재현 방법

다른 사람이 이 브랜치를 그대로 받아서 main과 동일한 명령으로 전체
시스템을 실행하고 최근 main 실행 결과와 비교할 수 있다:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch agconav_bringup agconav_sim.launch.py
```

월드, 로봇 스폰 위치, 속도/고도/경로, 물리엔진 설정 등 다른 어떤 기본
인자도 바꾸지 않았으므로 결과 비교는 순수하게 지도 누적/병합
알고리즘(칼만필터, 혼합모델, 분산 기반 선택)의 효과만 반영한다.
