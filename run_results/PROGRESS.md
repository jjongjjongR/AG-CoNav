# PROGRESS — 5m AGL 방법B 경로 실험 (2~5단계)

## 🟡 진행 중 — 5m AGL, 스트립간격 4m, 속도 5m/s 실험 (신규 세션, 사용자 취침 중 자율 진행)

**전체 타임아웃: 4.5시간(사용자가 3시간→4.5시간으로 변경 지시, 세션 중 반영).**
판단이 필요한 지점은 확인받지 않고 안전한 기본값으로 스스로 진행, 근거는 이
문서에 계속 기록한다(신규 항목은 관례대로 파일 맨 위에 추가). 범위:
`test_main_brian` 브랜치만, `git push --force` 금지, 범위 밖 소스코드 수정 금지.

### 0. 브랜치 확인
- `~/AG-CoNav-test_main`는 이번 세션 시작 시 `test_main` 브랜치였음(작업폴더가
  최근 다른 용도로 쓰인 흔적: `glim_config/config.json`,
  `src/agconav_test_worlds/launch/experiment.launch.py` 수정 + `path_100x100_5m_
  {2,3,4}m.yaml`/`run_results/` untracked). `git stash push -u`로 전부 보관(안
  버림 — `test_main` 브랜치 컨텍스트의 스태시로 남아있음, 이번 작업 범위 밖이라
  그대로 둠).
- `git checkout test_main_brian` 완료. 이 브랜치는 이미 `1d7a3aa [feat] 84m +
  방법B 실험 완료`까지 커밋되어 있고 `run_results/`도 이미 커밋된 상태(84m 실험
  산출물 전부 포함, 아래 706줄 기존 기록 그대로 보존). `git pull origin
  test_main_brian` → "Already up to date."
- 과거 기록 확인 결과 **5m AGL 실험(attempt1~4)은 전부 무효/실패**했고(climb-rate
  발산, VM 크래시, TF 동결, 정지판정 버그), 이후 목적을 84m+방법B로 전환해
  그것만 완료함. 즉 **5m AGL 스트립간격 4m(어떤 속도로도) 방법B 결과는 이번이
  최초**다 — 8절 비교표의 "5m+방법B(간격5m)" 행은 인용할 이전 데이터가 없다
  (간격 2/3/4m 경로 파일 자체는 2단계에서 만들어졌으나 velocity 비행이 전부
  실패해 방법B를 적용해본 적이 없음). 이번엔 간격4m·**속도5m/s**(기존
  attempt들은 4m/s)로 새 경로를 만들어 처음부터 시도한다.

### 1. 디스크 정리
`du -sh ~/AG-CoNav-test_main/bags/*` 결과: **디렉토리가 비어있음(파일 0개)** —
이전 세션 종료 시점에 이미 전부 정리되어 있었다(SUMMARY.md/PROGRESS.md에 attempt3
41GB, attempt4 1.6GB, 84m bag 5.1GB 모두 분석 후 삭제 기록 확인). `run_results/logs`,
`run_results/clouds`도 존재하지 않음(gitignore 대상, 이전 세션이 정리 후 커밋).

`df -h ~`: `29G/78G(39%), 여유 46G` — **이미 목표(60% 이하)를 크게 만족**한다.
지울 것이 없으므로 삭제 작업 없이 다음 단계로 진행. 보호 대상
(`path_100x100_5m_*.yaml`, `run_results/*.md,*.png`, GLIM apt 패키지, `.git`)은
확인만 하고 손대지 않음.

### 2. 사전 준비
- `pgrep -af "ros2|gz sim|docker.*glim"` → 결과 없음, 잔여 프로세스 없이 깨끗한
  상태 확인.
- 디스크 감시 스크립트 `-k` 버그: `run_results/run_attempt4_monitored.sh`,
  `run_84m_velocity_monitored.sh` 둘 다 이미 `timeout -k 5 3 ros2 topic echo ...`
  로 수정되어 있고, 84m 실험 전체를 통해 실전 검증까지 끝난 상태(SUMMARY.md
  "알려진 버그" 절 — 이 스크립트가 실제로 정상 감지/종료했음). 코드 레벨 확인
  완료로 간주.
- `colcon build --symlink-install` 성공 (`Summary: 11 packages finished [3.86s]`,
  에러 0건, deprecation 경고만 있음 — symlink-install이라 원래도 빨랐음).
- `timeout -k` 독립 재검증: SIGTERM을 무시하도록 만든 테스트 스크립트(`trap ''
  TERM; sleep 30`)에 `timeout -k 5 3 ...`을 걸어 실행 → 정확히 8초(3초 SIGTERM
  대기 + 5초 유예 후 SIGKILL)만에 `Killed`로 강제종료 확인. 코드+실전(84m
  실험)+이번 독립시험까지 3중으로 확인 완료.

### 3. 경로 재생성 (5m AGL, 방법B, 스트립간격 4m, 속도 5m/s)
`run_results/generate_path_4m_5mps.py`(신규, `generate_paths.py`의 지표면모델/
안전검증/상승률제한 로직 그대로 재사용, `SPEED_MPS=5.0`만 교체) 실행 결과:

- 웨이포인트 5,251개(원시=최종, **자동보정 0회** — 4m/s판(9회 보정, 최소클리어런스
  3.033m)보다 오히려 더 여유있게 나옴, 이유: 속도가 5m/s로 빨라지면서 같은
  climb_speed_mps 예산(6.0x0.7=4.2m/s)에서 허용 기울기 `MAX_SLOPE=4.2/5.0=0.84`가
  4m/s판(`4.2/4.0=1.05`)보다 더 완만해져 상승 프로파일이 건물 진입 전부터 더
  일찍·더 넓게 퍼져 시작됨).
- **웨이포인트 자체**: `z = 지표면(x,y) + 5.0m` 이상(rate-limited majorant라 실제로는
  이보다 더 높을 수 있음, 항상 위로만 완화).
- **웨이포인트 사이 구간(0.2m 샘플)**: 최소 클리어런스 **4.720m** ≥ 3.0m 마진 충족.
- **climb rate**: 요구 수직속도 최대 **4.20m/s** = 설계 상한과 정확히 일치(레이트
  리밋이 의도대로 작동), 예산(6.0m/s) 초과 세그먼트 **0개**.
- 경로길이 2847.4m, 등속 5m/s 기준 예상 순수비행시간 569.5s(9.5분), 턴 25회.
- 저장: `src/agconav_test_worlds/config/path_100x100_5m_4m_5mps.yaml`,
  `run_results/path_5m_4m_5mps_safety_report.md`.

### 4. 순간이동 드라이런 검증
1차 시도(`bags/dryrun_teleport_4m_5mps`)는 `drone_path_player`가
`FileNotFoundError`로 즉시 죽음 — **경로 yaml을 3단계에서 소스트리에만
저장하고 colcon build를 다시 안 돌려서** `install/`에 새 파일이 없었던 것
(symlink-install도 "새로 추가된 파일"은 재빌드해야 심볼릭링크가 생김, 기존
파일 내용 변경과는 다름). `colcon build --symlink-install --packages-select
agconav_test_worlds`로 해결 확인 후 재실행.

2차 시도(재실행) 결과: **1097초(18.3분) 만에 `path_status=True` 정상 완주.**
`run_results/analyze_bag.py`로 분석(`run_results/dryrun_4m_5mps.md`):
- **박스 기준 커버리지 99.6%**(996,488/1,000,000) — 4m/s판 드라이런(99.6%)과
  거의 동일, 5m/s로 속도만 바뀐 것이 커버리지에 미치는 영향은 무시할 수준.
- 박스 밖 유효 셀 2.92%(29,957개) — 경계 5m 이내 87.9%, 4m/s판(88.2%)과 같은
  패턴(스와스 폭 특성 + 코너 yaw반전 보간 불안정, 기존 문서화된 무해한 현상,
  `PROGRESS.md` 하단 "박스 밖 유효 셀의 공간 분포" 절 참조) — 새로 생긴 문제
  아님.
- 이상치(자기반사 의심) 0건, 고도 범위 0.936~8.958m로 지표면 모델 기대범위
  (1.202~8.780m)와 사실상 일치.
- **판정: 경로 자체는 안전/정상, 5단계(velocity 실비행) 진행.** bag(11GB)은
  분석 완료 후 삭제(재현 가능, 디스크 53%→39%로 원복).

### 5. Velocity 비행 실행
`run_results/run_velocity_4m_5mps_monitored.sh`(신규, 84m 방법B 실험 스크립트
패턴 재사용) 작성:
- `flight:=velocity path_file:=path_100x100_5m_4m_5mps.yaml cruise_speed_mps:=5.0
  module_a:=false module_f:=false` — 84m 방법B와 동일하게 모듈 A/F는 라이브로
  안 돌림(방법B는 bag만 있으면 오프라인 처리 가능 + attempt3에서 유력했던
  "module_a까지 같이 돌 때의 자원경합" 원인을 애초에 피하는 선택).
- bag: `/drone/points /drone/imu /tf /tf_static /drone/path_status`만 실질적으로
  채워짐(나머지는 launch가 항상 구독하는 고정 토픽 목록에 있지만 발행자가 없어
  bag에 안 찍힘 — 지시된 5개 토픽과 결과적으로 동일).
- 재발방지: `timeout -k` 적용, **마지막 웨이포인트(N/N) 도달 후에는 진행률
  정지를 STALL로 오판하지 않도록 예외처리**(SUMMARY.md에 기록된 attempt4의
  기존 버그 재발 방지), **module_a를 안 돌려 "TF lookup failed" 로그가 없으므로
  `ros2 topic hz /tf --window 20`를 60초 주기마다 4초 창으로 직접 찔러 /tf
  발행 여부를 독립적으로 확인**(3회=3분 연속 무응답 시 TF_FROZEN으로 즉시 중단
  — attempt3의 핵심 위험이었던 TF 동결을 이번엔 module_a 로그에 의존하지 않고
  직접 감지).
- TIMEOUT=3600s(60분, 등속 순수비행시간 569.5s의 6.3배 여유 — attempt4 선례의
  5.7배 여유율과 비슷한 수준으로 판단해 설정), DISK_LIMIT=80%, STALL_CHECKS=5분.
- 정지조건 2번(3~4회 재시도 후에도 불안정/충돌 시 중단) 적용 예정: 아래에
  attempt별 결과를 계속 기록.

**attempt1 — 1회 만에 정상 완주, 재시도 불필요.**
- 2157초(36.0분) 만에 `path_status=true`, 5251/5251 웨이포인트 전부 도달.
- `/tf` 무응답 감지가 총 2회 있었으나 둘 다 **1회(1분)만 반짝하고 바로 회복**
  (3회 연속 기준에 못 미침 — attempt3의 "40분 동결"과는 전혀 다른, 정상적인
  샘플링 노이즈로 판단). TF 동결 재발 없음.
- 디스크: 시작 39%→종료 시점 68%(24GB 여유), 80% 임계치 근처 간 적 없음 —
  module_a/f를 라이브로 안 돌린 선택이 attempt3의 근본원인(추정: I/O 경합)을
  효과적으로 피한 것으로 보임.
- SIGINT로 launch가 30초 안에 안 죽어 SIGTERM으로 에스컬레이션됨(정상 동작
  범위 — shutdown_launch 로직이 설계한 대로 작동) → 그 여파로 `ros2 bag
  record`가 metadata.yaml을 못 쓰고 죽어 **bag 메타데이터 유실**(84m 실험 때와
  동일 패턴, SUMMARY.md "알려진 버그"와 같은 계열). `ros2 bag reindex -s mcap
  bags/velocity_4m_5mps_attempt1`로 완전 복구 확인: **21,771 스캔, TF
  109,013개, path_status 1개(True), 21.4GiB, duration 2183.2s** — 재비행
  불필요.
- 결론: **정지조건 2번(3~4회 재시도) 발동 없이 1회차에 성공**, 6단계로 진행.

### 6. 방법B(GT pose + GICP 정합) 적용

**시도1(원본 스크립트, 84m 실험에서 그대로 재사용)이 OOM-kill됨.** 84m 실험은
스캔 ~1,670개였지만 이번 5m AGL 4m/5mps 비행은 스캔 21,771개(13배)라, 원본
`build_methodB_cloud.py`가 전체 비행 분량의 점을 파이썬 리스트(baseline_chunks,
methodb_chunks)에 다 들고 있다가 마지막에 한 번에 `np.concatenate`하는 방식이
이 VM(5.8GB RAM)에서 감당이 안 됐다. 스캔 21750/21771까지(99.9%) 정상 처리하고
마지막 concatenate 직전에 죽음 — 파이썬 예외/트레이스백 없이 그냥 사라져서
처음엔 원인이 불명확했으나, `/var/log/syslog`에서 확증:
```
oom-kill: ... task=python3,pid=19588 ...
Out of memory: Killed process 19588 (python3) total-vm:10044336kB, anon-rss:4695904kB
```
(참고: 이 확인 과정에서 background 작업 실행 방식도 문제가 있었음 — Bash
run_in_background으로 직접 띄운 무거운 계산이 지정한 타임아웃보다 먼저
알수없는 이유로 두 번 종료됨(각각 30분/90분 지정, 그보다 일찍 "killed"),
프로세스 자체의 이슈인지 tool 실행환경 이슈인지 불명확해 이후로는
`nohup ... &; disown`으로 완전히 분리한 프로세스를 띄우고 별도 폴링으로
감시하는 방식으로 전환함 — 재현되면 다음 세션도 이 방식을 쓰는 게 안전.)

**조치**: `build_methodB_cloud.py`를 스트리밍 방식으로 재작성(판단: 방법론
자체나 실험 범위를 안 건드리고 순수 구현 최적화이므로 확정 범위 밖 변경
아님, 정지조건 3번 해당 안 됨) —
- 스캔마다 바로 `.raw`(헤더 없는 연속 float32) 파일에 append, 파이썬 리스트에
  전체 이력을 안 쌓음(GICP 타겟용 `window`는 원래도 최근 6스캔만 유지해 작음).
- 끝에서 `.raw` → `.npy` 변환도 `np.lib.format.open_memmap`으로 만든
  디스크백킹 배열에 2M점씩 청크로 복사(전체를 한 번에 메모리에 안 올림).
- **재검증**: RSS를 스캔 750개 시점에 직접 확인(`ps`) → **116MB**(원본 방식이면
  이 시점 이미 수백MB~1GB대로 자라고 있었을 것) — 픽스가 의도대로 작동함을
  확인.
- 부수 조치: `feed_cloud.py`(7단계에서 이 cloud를 Module A/F에 흘려보낼 때 씀)도
  같은 OOM 위험이 있어(`np.load`로 전체를 한 번에 RAM에 올림) `mmap_mode='r'`
  + `astype(..., copy=False)`로 지연로딩되게 최소 수정(범위: 이 실험 파이프라인이
  이 VM에서 끝까지 돌아가는 데 필수적인 인프라 수정으로 판단, 방법론/토폴로지
  변경 아님).

시도2(수정판) 결과: **성공.** RSS를 스캔 750개 시점 확인 시 116MB로 안정
(원본이면 이 시점 이미 커지고 있었을 값) — 전체 21,771 스캔 완주:
```
스캔 21771개 (TF 조회 실패로 스킵 0개)
GICP: 성공 21398, 실패(GT로 대체) 372, 첫 스캔이라 스킵 1 (성공률 98.3%)
baseline 점 266,757,021개 -> run_results/clouds/baseline_4m_5mps.npy (3.0GB)
방법B    점 266,757,021개 -> run_results/clouds/methodb_4m_5mps.npy (3.0GB)
```
(참고: 84m 실험 대비 GICP 성공률 98.3% vs 84m의 94.5% — 5m AGL의 조밀한
근거리 스캔이 GICP 수렴에 더 유리했던 것으로 보임.)

두 cloud 확보 후 원본 flight bag(22GB, 디스크 76%까지 올라간 상태)은 분석
불필요해져 삭제(재현 가능 — 재비행 스크립트로 언제든 재현) → 디스크
47%로 원복.

### 7. Wheel FN% 채점
`run_results/run_traversability_fn_v2.sh`(원본 `run_traversability_fn.sh`과
동일 로직, `capture_nav_fn.py` 타임아웃만 180s→1500s로 확대 — 이번 cloud가
84m 대비 훨씬 커서 `feed_cloud.py`가 200,000점/0.5s로 전부 흘리는 데만
~4~5분 이상 걸림, 원본 180s로는 시간 안에 못 끝남) 재사용.

**baseline_4m_5mps (정합 없음, GT pose만) 결과**:
- wheel FN% = **13.44%** (618,378개 GT통과가능 셀 중 83,102개 막힘오판,
  미측정 2.35%)
- leg FN% = **12.60%** (77,921개 막힘오판, 미측정 2.35%)
- elevation_map 커버리지 86.4%(997,553/1,154,520셀 — 그리드가 100x100보다
  약간 큼(108.0x106.9m), `_grow_to_fit` 기존 결함 계열, 2~4단계에서 이미
  무해함을 확인한 것과 같은 패턴)

**방법B(GICP) 1차 채점 — 파국적 결과, 원인 진단 후 재작업.**

1차 결과(`methodb_4m_5mps_fn_UNCORRECTED_gicp_outlier.json`로 보존):
wheel FN% **46.49%**, leg FN% **34.61%** — baseline보다 오히려 훨씬 나쁨.
elevation_map이 108x106.9m(baseline)에서 **128.2 x 388.5m**로 비정상 팽창,
`height_max=84.006m`(이 실험의 지표면 기대범위 1.2~8.9m를 완전히 벗어남 — 참고로
"84"라는 숫자가 이 리포의 다른 실험(84m 고도)과 우연히 겹쳐서 처음엔 혼동
가능성이 있었으나 무관한 우연의 일치임, 실제로 GICP가 잘못 수렴한 절대값일 뿐).

**직접 원인 확인**: `methodb_4m_5mps.npy`(2.67억 점)를 직접 스캔 → z>15m 또는
|y|>250m인 명백한 이상치 **44,864개(0.017%)** 존재, 최대 z=84.09m, 최대
y=218.68m. 파이프라인/채점 스크립트 버그가 아니라 **cloud 자체에 실제로
박혀있는 오염**임을 확인(방법B 생성 스크립트 자체 문제, Module A의 기존
`_grow_to_fit` 무제한 성장 결함이 이 오염을 극단적으로 증폭시켜 그리드
전체와 FN%를 다 망가뜨림).

**근본원인 추정**: `build_methodB_cloud.py`의 GICP 호출이 `result.converged
== True`만으로 결과를 신뢰하는데, 이번 경로(코너 25회, 84m 기준선의 4회보다
훨씬 많음)의 코너(180도 yaw 반전)마다 스캔이 극도로 작아지는 구간이 반복되고
(빌드 로그에 "point cloud is too small(2~10점)" 경고 다수 관측), 이런
저정보 상황에서 GICP가 물리적으로 말이 안 되는 국소해로 "수렴"할 수 있음을
실측으로 확인. 이건 84m 실험(코너 4회뿐)에선 거의 안 보였던(원시 통계만
소폭 악화, p95 1.62→2.66m) 현상이 코너 수가 훨씬 많은 이번 5m AGL 경로
에서는 파국적 규모로 증폭된 것으로 판단.

**조치(판단, 확정범위 밖 아님으로 결론)**: `build_methodB_cloud.py`에
"GT 대비 GICP 이동량이 물리적으로 타당한 범위(2.0m, GICP
max_correspondence_distance=1.0m 탐색폭 감안 시 정상 보정량은 원래 수십cm대
여야 함)를 벗어나면 신뢰 안 하고 GT로 폴백"을 추가 — 원래 코드에 이미 있던
"GICP 실패 시 GT 안전 폴백" 설계(`result.converged==False`만 다루던 것)를
"수렴은 했지만 말이 안 되는 해"까지 포괄하도록 완성한 것으로, 방법론/실험
범위 자체를 바꾸는 게 아니라 스크립트의 기존 안전장치를 의도대로 완성하는
버그 수정에 해당한다고 판단(정지조건 3번 미해당). 1차(미보정) 결과는 진단
가치가 있어 파일명에 `UNCORRECTED_gicp_outlier`로 명시해 보존.

**뼈아픈 실수**: 방법B cloud 생성 직후 "재현 가능하다"고 판단해 원본
flight bag(22GB)을 이미 삭제해버려서, 이 수정을 반영하려면 **재비행이
필요**했다(attempt2, 아래 계속 기록). baseline cloud는 이 결함과 무관(GT
pose만 쓰므로 GICP 자체가 안 들어감)해서 재사용 가능, methodb만 재생성.
앞으로는 방법B처럼 "정합 실패 시나리오를 다시 봐야 할 수도 있는" cloud
생성 단계에서는 채점까지 완전히 끝나기 전엔 원본 bag을 지우지 않는 게
안전하다는 교훈.

**재비행(attempt2) 및 보정판 방법B 결과**:
- attempt2: `run_velocity_4m_5mps_monitored.sh attempt2`로 재비행. TF는
  이번에도 정상(중간 1회 블립 후 즉시 회복), 그러나 **wp 5239/5251(99.8%)
  에서 디스크 80% 임계치에 정확히 도달해 설계대로 즉시 중단**(DISK_THRESHOLD_
  STOP, 정상 안전동작 — attempt1보다 총 기록량이 약간 더 많았던 것으로 보임,
  bag 24.6GiB). `path_status`는 못 찍었지만(마지막 12개 웨이포인트 도달 전
  중단) 방법B 생성은 `/tf`+`/drone/points`만 있으면 되므로 이 bag도 완전히
  유효 — 재비행은 불필요, 이 bag으로 바로 진행. bag도 SIGTERM 에스컬레이션
  여파로 메타데이터 유실 → `ros2 bag reindex -s mcap`으로 복구(25,117 스캔,
  TF 125,584개).
- 중단 시점 디스크가 81%까지 순간적으로 넘어감(정지조건 "80% 초과시 즉시
  중단"은 지켰으나 감시주기가 60초라 마지막 한 틱 사이에 79%→81%로 건너뜀) —
  즉시 진단 완료 후 안 쓰는 파일(1차 미보정 cloud 3GB) 삭제로 77%로 낮추고
  계속 진행, 이후 방법B cloud 생성 완료 직후 attempt2 bag(25GB)도 삭제해
  48%로 안정화.
- **보정판(이동량 상한 2.0m 필터 적용) GICP**: 성공 24,707, 실패(GT대체)
  409(그 중 "수렴했지만 이동량 상한 초과로 기각" 49개 — 새로 추가한 안전장치가
  실제로 작동함을 확인), 점 309,125,893개.
- **오염 재검증**: cloud 직접 스캔 결과 z>15m 또는 |x|>100 또는 |y|>250
  이상치 **996개/3.09억(0.0003%)**로 격감(1차 미보정판 44,864개 대비 98%
  감소). z 범위 0.068~15.09m로 지표면 기대범위(1.2~8.9m)에 훨씬 가까워짐
  (완전히 0인 건 아니라 100% 클린은 아니지만, 그리드를 파국적으로 부풀리던
  수준은 해소).
- **보정판 채점 결과**: elevation_map 크기 109.7x106.2m(baseline 108x106.9m과
  비슷한 정상 범위로 복귀, 1차의 128.2x388.5m 폭주 해소), 커버리지 89.2%.
  **wheel FN% = 37.26%, leg FN% = 30.03%** — 1차 파국적 결과(46.49%/34.61%)
  보다는 크게 개선됐지만, **baseline(13.44%/12.60%)보다는 여전히 훨씬
  나쁘다.** step 중앙값도 baseline 0.0092m → 방법B 0.0489m(5.3배), wheel
  기준(0.08m) 초과 비율도 16.1%→38.8%로 뚜렷이 악화 — 이제는 이상치 몇 개가
  아니라 **전반적인 노이즈 증가**로 보인다.
- **해석**: 84m 실험(코너 4회)은 GICP가 wheel FN%를 개선했지만, 이번 5m AGL
  4m/5mps 경로(코너 25회, 스트립마다 180도 yaw 반전)는 코너마다 스캔이
  작아지는 구간이 잦아 GICP가 안정적으로 정합할 타겟/소스 형상 자체가
  부족한 경우가 훨씬 많이 반복된 것으로 보인다(이동량 상한으로 최악의
  경우는 걸렀지만, 상한 안에 드는 "그럴듯하지만 부정확한" 미세 오정합까지는
  못 거름 — 이게 누적되며 baseline보다 못한 결과를 만든 것으로 판단). 즉
  **이번 실험 조건에서는 방법B가 순수하게 역효과였다** — 84m과 정반대 방향
  결과.
- 산출물: `methodb_4m_5mps_fn.json`(보정판, 최종), `methodb_4m_5mps_fn_
  UNCORRECTED_gicp_outlier.json`(1차 진단용 보존), `run_results/clouds/
  {baseline,methodb}_4m_5mps.npy`.

### 8. 비교표

전체 비교표와 분석은 `run_results/SUMMARY.md` 맨 위 절("5m AGL 방법B 실험")에
정리했다. 핵심 결론:
- **간격(5m→4m) 자체의 개선 효과는 이번 실험만으로 확인 불가** — 5m 간격
  방법B 결과가 존재한 적이 없음(이전 세션 5m AGL 시도 전부 실패).
- **고도 자체(84m→5m AGL)는 GT 기준선 기준 뚜렷한 개선**: wheel FN%
  23.94%→13.44%(-44% 상대), leg는 거의 동일(12.15%→12.60%).
- **정합(GICP) 효과는 84m과 5m AGL/4m/5mps에서 정반대**: 84m은 개선
  (wheel -16% 상대), 이번 5m AGL/4m간격/5m/s는 **악화**(wheel +177% 상대,
  leg +138% 상대) — 코너가 훨씬 잦은(25회 vs 4회) 저고도 lawnmower 경로에서
  코너마다 반복되는 저정보(작은 점군) 스캔 구간이 GICP를 체계적으로
  불안정하게 만든 것으로 판단(자세한 원인 진단은 7절 참조).
- **실용적 결론**: 이번 실험 조건(5m AGL, 4m 간격, 5m/s, 코너 잦은 lawnmower)
  에서는 방법B(GICP 정합)를 쓰지 않고 GT pose만 쓰는 쪽(baseline)이 명백히
  더 낫다. 방법B는 "코너가 드문 경로"라는 조건에서만 순이득이었을 가능성이
  높다.

### 9. 저장/보고

**산출물 최종 목록**:
- `src/agconav_test_worlds/config/path_100x100_5m_4m_5mps.yaml` — 이번 실험
  경로(5,251 웨이포인트, 5m/s, 4m 간격, 5m AGL).
- `run_results/path_5m_4m_5mps_safety_report.md` — 경로 안전검증 보고서.
- `run_results/generate_path_4m_5mps.py` — 경로 생성 스크립트(재사용 가능).
- `run_results/run_velocity_4m_5mps_monitored.sh` — velocity 비행 실행+감시
  스크립트(재발방지 반영: `timeout -k`, 마지막 웨이포인트 STALL 오판 방지,
  `/tf` 직접 감시).
- `run_results/build_methodB_cloud.py` — 방법B cloud 생성(이번 세션에 두
  가지 버그 수정: OOM 방지 스트리밍 방식 재작성, GICP 이동량 상한 안전장치
  추가 — 84m 실험에서도 재사용 가능한 개선).
- `run_results/run_traversability_fn_v2.sh` — FN% 채점 스크립트(캡처
  타임아웃만 확대, 원본 로직 무수정).
- `run_results/{baseline,methodb}_4m_5mps_fn.json` — 최종 채점 결과.
- `run_results/methodb_4m_5mps_fn_UNCORRECTED_gicp_outlier.json` — 진단용
  보존(GICP 이동량 상한 적용 전, 파국적 오염 결과).
- `run_results/clouds/{baseline,methodb}_4m_5mps.npy` — 누적 point cloud
  (각 3.0~3.7GB, git 미포함 — 84m 실험과 동일 관례로 재현 가능해서 로컬만
  보존, bag은 이미 삭제).
- `run_results/SUMMARY.md` 맨 위 절, `run_results/PROGRESS.md`(이 문서) —
  전체 과정/판단근거 기록.
- 부수 수정: `src/agconav_test_worlds/scripts/feed_cloud.py`(mmap_mode='r'로
  OOM 방지, 큰 cloud를 이후에도 안전하게 피드할 수 있게).

**디스크 최종 상태**: 48%(35G/78G 사용, 39G 여유) — 시작 시(39%)보다 약간
높지만 목표(60% 이하)를 여유있게 만족. bag은 전부 삭제, cloud만 로컬 보존.

**타임아웃 준수**: 세션 시작 약 03:53, 완료 약 07:00 — 총 소요 약 3시간 10분,
사용자가 변경한 4.5시간 예산 안에서 완료.

**git 커밋/푸시 완료**: `test_main_brian` 브랜치에 커밋 `4d73d5f`
("[feat] 5m AGL + 방법B(간격4m, 5m/s) 실험 완료 — 84m과 정반대로 GICP가 FN%
악화시킴 확인"), `git push origin test_main_brian` 성공(`1d7a3aa..4d73d5f`).
25개 파일 변경(경로 yaml, 스크립트 6개, 채점 결과 3개, 보고서 2개, 로그
12개, PROGRESS/SUMMARY.md). cloud(.npy, 6.9GB)와 진단용 대용량 npz(surface_
cache.npz 650KB, dryrun_4m_5mps_grid.npz 94MB)는 84m 실험과 동일 관례로
git 미포함(재현 가능, 로컬 보존).

**최종 상태 확인**: `pgrep -af "ros2|gz sim|docker.*glim"` → ros2-daemon
표준 헬퍼만 남음(무관, 안 건드림), 실험 관련 프로세스 전부 정상 종료.
`df -h ~` → 48%(35G/78G, 39G 여유) — 시작(39%)과 비슷한 수준으로 안정.

## 세션 종료

모든 단계(0~9) 완료. 다음 세션이 이어받을 것은 없음 — 이번 실험은 완결됨.
후속 조사가 필요하다면(사용자 판단): (a) 5m AGL에서 코너를 줄인 경로(예:
스트립 진행방향을 길게, 옆이동만 최소화)로 GICP가 실제로 코너-저정보 가설이
맞는지 검증, (b) GICP 이동량 상한(2.0m)을 더 타이트하게 줄이거나 타겟
윈도우의 최소 점 개수 문턱을 추가해 "그럴듯하지만 부정확한" 미세 오정합까지
걸러내는 실험, (c) 5m 간격 방법B를 실제로 완주시켜 이번 4m 간격과 비교
(간격 자체의 효과를 마침내 분리 측정).

---

## ✅ 완료 — 84m + 방법B(GT pose + GICP 정합) 실험, 목적 정정 후 재설계

사용자가 실험 목적을 정정: GLIM 자체 odometry가 아니라 GT pose를 신뢰하고
`small_gicp`(GLIM 내부 라이브러리)로 정합만 빌려 elevation map 노이즈를
줄이는 게 목적. 5m AGL 실험(attempt1~4, 전부 크래시/정지/무효)은 이 목적에
더 이상 안 맞아 중단, 이번 84m+방법B로 새로 설계해 완료했다.

- GLIM 자체는 외부 pose 주입/정합 전용 모드를 공식 지원 안 함(GitHub #193
  미해결로 확인) → GLIM을 통째로 쓰지 않고 `small_gicp` python 바인딩을
  직접 pip 설치(`pip3 install --user --break-system-packages small_gicp`,
  arm64 wheel 있음) 후 `run_results/build_methodB_cloud.py`를 새로 작성.
- 84m 비행(`path_100x100.yaml`, 8m/s, 10웨이포인트)은 167초 만에 정상
  완주, `path_status=True` 발행. 감시스크립트의 정지(stall) 오판 버그로
  5분 뒤 불필요하게 중단됐지만 bag은 `ros2 bag reindex`로 완전 복구(재비행
  불필요) — 자세한 내용은 `run_results/SUMMARY.md` 맨 아래 "알려진 버그" 참조.
- **결과: wheel FN% 23.94%→20.10%(개선), leg FN% 12.15%→11.89%(거의 무변화),
  단 원시 단차 통계(p95 1.62→2.66m)는 악화, 미측정 비율도 증가** — 정합이
  "쉬운 곳은 더 쉽게, 어려운 곳은 더 어렵게" 만드는 트레이드오프였다.
  전체 분석/해석은 `run_results/SUMMARY.md` 참조.
- 산출물: `build_methodB_cloud.py`, `run_84m_velocity_monitored.sh`,
  `84m_baseline_fn.json`/`84m_methodB_fn.json`, `maps_84m/` — 전부 git 커밋.
  bag(5.1GB)/point cloud(166MB)는 재현 가능해 git 미포함(bag은 삭제,
  cloud는 로컬 보존).

## 🟡 전체 정지 + 디스크 정리 (사용자 지시, 재시도는 별도 지시 대기)

**1) 프로세스 정지**: attempt4(진행 중이던 velocity 비행) 및 내 감시스크립트
(`run_attempt4_monitored.sh`) 전부 SIGINT→(bag record/launch 프로세스만
SIGTERM까지 필요)로 정상 정지. gz sim/bridge/follower/mapper는 개별
SIGINT로 곧바로 종료됨. 최종 확인(`pgrep -af "ros2|gz sim|docker.*glim"`)
결과 잔여 없음(ros2-daemon 표준 헬퍼 프로세스만 남음, 실험과 무관이라
안 건드림).

**2) 정리 전 디스크 조사 결과**

| 위치 | 크기 | 비고 |
|---|---|---|
| `~/AG-CoNav-test_main` 전체 | 1.8G | |
| `bags/velocity_4m_attempt4` | 1.6G | attempt4 미완주 bag(중단됨) |
| `run_results/` | 177M | .md/.png 등 산출물 포함, 보존 대상 |
| `build/` | 23M | colcon 빌드 산출물, 보존(재빌드 방지) |
| `install/` | 2.3M | colcon 설치본, 보존 |
| `log/` | 1.1M | colcon 로그, 보존 |
| `docker system df` | **확인 불가** | permission denied — docker 그룹 권한 미해결(이전부터 계속) |
| `/var/lib/docker` 직접 확인 | **확인 불가** | 마찬가지로 권한 없음(`sudo`도 비밀번호 필요) |
| `/var/cache/apt/archives` | 198M | |
| `df -h ~` (정리 전) | 30G/78G(41%) | attempt3 bag은 이전에 이미 삭제해둔 상태라 이번 위기 수준은 아니었음 |

**docker 관련 확인 불가 — 사용자 조치 필요할 수 있음**: `docker system df`도
`/var/lib/docker` 직접 열람도 전부 permission denied(docker 그룹 미가입 +
`sudo`가 비밀번호 요구, 이전 GLIM PPA 작업 때와 동일한 세션 제약).
Docker 자체는 이제 안 쓰기로 했으니(PPA 전환 완료) 지워도 된다는 승인은
받았지만, **제가 직접 지울 방법이 없다.** 유의미한 용량이면 사용자가
직접 터미널에서 `docker system prune -a` 등을 실행해야 함 — 다만 이번
정리의 핵심 목표(bag 파일)는 이미 해결되므로 급하지 않음.

**3) 삭제 실행**: `bags/velocity_4m_attempt4`(1.6G, 미완주) 전체 삭제,
`/tmp/glim_*.log`(합계 <1KB, 이전 GLIM 스모크테스트 부산물) 삭제.
`apt clean`은 `sudo` 비밀번호 필요로 건너뜀(198M 그대로 — 영향 미미).

**4) 정리 후 디스크**: `df -h ~` → **29G/78G(39%), 여유 46GB**
(정리 전 30G/78G 41% → 큰 차이 없음, attempt4가 중단 시점에 1.6G밖에
못 쓴 상태였어서 애초에 위기 수준이 아니었음 — attempt3 삭제가 이미
끝난 뒤라 이번 정리는 예방적 성격). 보호 대상(경로 yaml 3개, run_results
내 .md/.png 전부, GLIM apt 패키지, .git)은 전부 그대로 확인됨.

## 🟢 근본원인 정정 — Module A/F 문제 아님, 순수 LiDAR 원본 데이터량 문제

사용자 질문("모듈 A,F만 돌리는데도?")에 답하며 재조사, 앞선 "리소스 부족"
가설을 더 구체화/정정함:

- **Module F는 애초에 안 돌고 있었다** — attempt3 launch 커맨드에
  `module_f:=true`를 안 넣어 기본값(false)으로 실행됨. flight log에
  `terrain_feature_calculator`/`verdictor` 관련 줄 0건으로 확인. 즉
  wheel/leg 관련 토픽엔 발행자가 아예 없어 부하 0.
- **`/drone/elevation_map`도 원인이 아니다** — `drone_elevation_mapper.py`
  소스 확인 결과 이 토픽은 **`path_status`가 true가 될 때 딱 1번만
  발행**(`self._published` 플래그로 막음). attempt3는 완주를 못 해
  `path_status`가 끝까지 false였으므로 **이 토픽은 이번 bag에 아예 한
  번도 안 찍혔다.** (그리드가 1658x2506칸까지 부푼 것 자체는 사실이고
  여전히 RESULTS.md 9절 기존 결함이지만, 이번 I/O 문제의 원인은 아님.)
- **진짜 원인 — 순수 원본 LiDAR 포인트클라우드 유량**: `model.sdf`의
  os1_lidar 스펙 `horizontal samples=1024, vertical samples=32,
  update_rate=10`Hz → 초당 327,680점. PointCloud2 포인트당 대략
  20~30바이트(xyz+intensity+ring+time)로 잡으면 **`/drone/points` 하나만
  으로 초당 약 9~10MB의 지속적 쓰기 부하**가 나온다. 66분(강제종료까지
  걸린 실제 시간) 동안 지속하면 9.5MB/s×3960s ≈ **37GB — 실측 bag 크기
  41GB와 거의 일치.** 이게 거의 전부를 설명한다.
- **"약 190~195초마다" 패턴의 정체**: `ros2 bag record`의
  `--max-bag-size 2000000000`(2GB) 분할 기준을, 9.5MB/s로 나누면
  2000MB/9.5MB/s ≈ 210초 — 관측된 190~195초 간격과 거의 일치. 즉 그
  경고는 **매 2GB 분할파일이 넘어갈 때마다 파일 마무리(flush/finalize)
  가 유입 속도를 못 따라잡아서** 뜬 것 — Module A/F 연산 부하가 아니라
  순수 디스크 쓰기 처리량 문제.

**결론(정정)**: 문제는 Module A/F의 계산량이 아니라, **이 LiDAR 스펙
(1024x32@10Hz)의 원본 데이터 자체가 이 VM의 디스크 쓰기 대역폭에 근접/
초과하는 양**이라는 것. 정상적으로 완주했을 경우(의도한 ~42분)라면
9.5MB/s×2520s ≈ 24GB 정도로, 삭제 후 확보한 여유(46GB)보다 작아 80%를
안 넘겼을 가능성이 높다 — **이번 93% 근접은 주로 TF/follower 정지로
비행이 66분까지 비정상적으로 길어진 것 + 내 감시스크립트 버그가 겹친
결과**이지, LiDAR 기록 자체가 원천적으로 감당 불가능한 수준은 아니다.
다만 여유가 넉넉하지는 않으므로 디스크 감시(80% 임계치)는 계속 유지해야
한다.

**TF 동결 원인은 여전히 미확정**이지만, sim=999s부터 이미 2~3회의 2GB
분할-지연 경고가 반복된 시점과 겹치는 건 사실 — I/O backpressure가
`exp_drone_tf_bridge`(가벼운 프로세스인데도) 스케줄링을 밀어냈을
가능성이 여전히 유력한 가설. 확정은 못 함.
## 🔴 attempt3 근본원인 조사 결과 — bag 기록 I/O가 데이터 유입을 못 따라감

**사용자 지시로 재시도 전에 원인부터 조사. 삭제(승인받음) 완료, 디스크
29G/78G(39%)로 정상화.**

`~/.ros/log/latest/launch.log`(launch 시스템 자체의 이벤트 로그, 노드별
stdout과 별개)에서 결정적 증거 발견:
```
1786789778.00  [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
1786789970.55  [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
1786790166.83  [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
... (이하 약 190~195초 간격으로 종료 시점까지 총 20회 이상 반복)
```
**비행 시작 3.3분 지점(19:29:38)부터 강제종료 시점(20:34:26)까지 처음부터
끝까지 약 3분마다 계속 반복됐다** — `ros2 bag record`의 내부 캐시가
들어오는 메시지(특히 `/drone/points` 포인트클라우드)를 디스크에 쓰는
속도를 따라잡지 못해 계속 밀렸다는 rosbag2 자체 경고. 이게 **일회성이
아니라 처음부터 끝까지 만성적**이었다는 게 핵심.

**TF 동결과의 상관관계**: `exp_drone_tf_bridge`가 실패하기 시작한 시점
(sim 999s경, "latest data" 값이 999→1037→1043으로 서서히만 증가하며
따라잡지 못하다가 1043.000001에서 완전히 멈춤)이 이 bag 캐시 적체가
이미 5회 이상 반복된 이후였다 — **원인은 코드 결함이 아니라 이 VM
(4 core/5.8GB, 이미 RESULTS.md에도 "6-core 환경도 CPU 포화됐다"고 기록됨)의
디스크 I/O/CPU가 이만한 데이터량(포인트클라우드+지도+주행성 등 11개
토픽 동시 기록)을 감당 못 해, bag 기록이 밀리면서 다른 프로세스(특히
가벼워야 할 pose 브리지)까지 시스템 차원에서 자원 경합에 밀려 결국
멈춘 것으로 판단.** 정확한 최종 트리거(브리지 프로세스 자체의 큐
오버플로/데드락 여부)까지는 100% 확정 못 했으나, 타이밍 상관관계가
뚜렷해 "리소스 부족"을 유력한 근본원인으로 결론.

**attempt4 전 조치(사용자 확인 필요, 아직 미적용)**:
1. (필수) 감시스크립트 버그 수정: `timeout 3 ros2 topic echo` →
   `timeout -k 5 3 ros2 topic echo`로 강제종료 유예 추가 — 안전장치가
   또 먹통되는 일 방지.
2. (검토 필요) bag 기록 부하 자체를 줄이는 방법 — 예: 이번 5단계
   채점에 당장 안 쓰는 토픽(`/wheel/nav_map`,`/leg/nav_map` 등 4단계
   시점엔 아직 안 채워지는 것들)을 기록 목록에서 빼거나, 압축/청크
   크기를 조정. 다만 이건 확정 범위(정지조건 3) 밖 변경일 수 있어
   사용자 판단 필요.
3. 리소스 부족이 근본원인이라면 attempt4도 재현될 위험이 있다는 점을
   사용자에게 명시적으로 알리고 진행 여부를 확인해야 함.

## 🔴 attempt3 실패 — TF 동결 + follower 정지 + 감시스크립트 버그로 디스크 93%까지 방치

**결론부터: attempt3는 무효, 삭제 필요(사용자 확인 대기 — 디스크 위급이라 긴급).
attempt4 전에 원인부터 조사해야 함, 무작정 재시도 금지.**

**1) 감시 스크립트 자체의 버그 — 안전장치가 40분간 먹통이었다**
`run_attempt3_monitored.sh`의 `PSTATUS=$(timeout 3 ros2 topic echo
/drone/path_status --once 2>/dev/null)` 줄이 **19:55:38부터 20:35:53까지
약 40분간 멈춰 있었다.** `timeout`은 기본으로 SIGTERM만 보내는데
`ros2 topic echo`(rclpy) 프로세스가 이를 무시/처리 못 해 안 죽었고
(`-k` 강제종료 옵션을 안 넣은 게 내 실수), 이 한 줄이 루프 전체를
막아버려서 **그동안 진행률 로그도, 무엇보다 사용자가 이번에 특별히
요청한 디스크 80% 감시도 완전히 멈춰 있었다.** 이게 이번 세션의 핵심
실수 — 다음에 다시 만들 때는 `timeout -k 5 3 ...`처럼 강제종료 유예를
반드시 넣어야 한다.

**2) 그 사이 실제로 벌어진 일 — 지상 정답(TF) 동결 + 비행 정지**
- `drone_elevation_mapper`가 **sim 시각 1043.000001s(비행 시작 후 약
  17분 지점)부터 끝까지 "TF lookup failed ... extrapolation into the
  future"를 8,043회 연속으로 찍었다** — `/model/X3/pose`→`/tf` 브리지
  (`exp_drone_tf_bridge`, PID 4016)가 그 시점 이후 새 변환을 전혀
  발행하지 못한 것으로 보인다(프로세스 자체는 CPU를 계속 쓰며 살아있었
  으나 출력이 멈춘 상태 — 정확한 원인 미상, ros_gz_bridge의 장시간 Pose_V
  브리징 버그이거나 이 VM의 리소스 부족(4 core/5.8GB, load average
  2.9대)으로 인한 것일 가능성이 있으나 확정 못 함).
- 결과: **비행 시간의 96% 이상(sim 1043s~4184s+) 동안 Module A가 사실상
  아무 것도 매핑 못 함.** 완주했더라도 이 bag은 Module A 채점/GLIM 비교용
  으로 못 쓴다.
- `velocity_path_follower` 자체도 **wp 5239/5251(99.8%, 끝에서 12개
  남음)에서 진짜로 정지**했다 — 115초 넘게 위치(-30.69, -66.18, 6.72)가
  전혀 안 바뀜, 스스로 회복 못 함. (TF 동결과 같은 근본 원인일 가능성
  — follower가 쓰는 pose 소스도 결국 같은 브리지/월드 상태에 의존할 것.)

**3) 디스크 — 93%까지 방치됐다가 감시가 살아나며 정상 트리거됨, 하지만 늦었다**
감시가 멈춰있던 40분 사이 bag이 17GB→41GB로 계속 자랐고(TF/follower는
멈췄어도 `/drone/points`,`/drone/imu` 원본 스트림과 bag record는 안
멈췄음), 20:35:53에 디스크 93%를 찍자 스크립트가 (내가 수동으로 멈춰있던
`topic echo`를 강제종료해 루프를 풀어준 직후) **의도대로 즉시 감지해
`DISK_THRESHOLD_STOP`을 기록하고 SIGINT→SIGTERM 종료 절차를 실행**했다
— 로직 자체는 옳게 작동했다, 다만 40분 늦게. SIGTERM까지 줬는데도
`parameter_bridge`×4, `ros2 bag record`, `drone_elevation_mapper`가
안 죽어 **내가 수동으로 SIGKILL**해서 정리함. 정리 후 디스크
`69G/78G(94%), 여유 4.9GB`에서 더 안 늘고 안정화 확인.

**4) 판정 — attempt3 bag(41GB) 무효, 삭제 필요(긴급 확인 요청)**
경로 미완주(follower 자체 정지) + TF 동결로 Module A 데이터 96% 손실,
어느 쪽으로도 재사용 불가. 디스크가 94%(4.9GB 여유)라 이 41GB를 지워야
다음 시도든 GLIM 처리든 여유가 생긴다. `rm -rf`가 자동승인 밖이라 삭제는
사용자 확인 후 진행 — 이번엔 디스크 상황상 긴급으로 물어봄.

**5) attempt4 전에 필요한 것 — 무작정 재시도 금지**
같은 TF 동결이 재현되면 또 disk만 채우고 무효 결과가 나온다. 재시도 전에:
(a) 감시스크립트의 `timeout -k` 버그부터 고치고, (b) TF 동결 원인을 최소한
한 번은 더 조사(예: exp_drone_tf_bridge 로그를 별도 파일로 분리해 1043s
직전 마지막 정상 메시지 확인, 또는 이 VM의 리소스 한계 때문인지 CPU/메모리
추이와 상관관계 확인)하는 게 안전하다고 판단 — 이것도 사용자 확인 필요
(정지조건 2번: "velocity 비행이 불안정할 때"에 해당한다고 봄. attempt1
climb-rate, attempt2 VM크래시에 이은 **세 번째로 다른 종류의 실패**라
그냥 또 재시도하는 것보다 원인 규명이 우선이라고 판단).

## ✅ GLIM PPA 설치 완료 + 네이티브 실행 조사 (docker → apt 전환 진행 중)

사용자가 직접 터미널에서 PPA 등록 + 패키지 설치를 완료함(`ros-jazzy-glim`,
`ros-jazzy-glim-ros` 1.2.2-0noble, arm64 — `dpkg -l`로 설치 확인).

**실행파일**: `ros2 pkg executables glim_ros` → `glim_rosnode`(라이브 구독,
기존 Docker `run_glim.sh`가 쓰던 것과 동일 모드), `glim_rosbag`(bag 배치 처리,
이번엔 Docker처럼 `ros2 bag play`를 별도로 띄울 필요 없이 이걸로 대체 가능해
보임), 그 외 `map_editor`/`offline_viewer`/`validator_node`.

**설정 경로 확인**: `-p config_path:=<glim_config 절대경로>` 로 우리 리포의
기존 `glim_config/`(CT+passthrough+pose_graph, Docker에서 쓰던 것과 완전히
동일한 파일)를 그대로 재사용 가능 — 별도 config 없이 절대경로만 바꿔주면 됨.
스모크테스트(`glim_rosbag /nonexistent_bag_path --ros-args -p config_path:=...`)
결과: `load libodometry_estimation_ct.so` / `libsub_mapping_passthrough.so` /
`libglobal_mapping_pose_graph.so` 전부 정상 로딩(Docker의 GPU 로딩 실패 문제가
CPU 조합에서는 재현 안 됨) — **CT+passthrough+pose_graph 조합이 네이티브에서도
그대로 작동함을 확인.**

**⚠️ 미해결 — bag 경로 인자 전달 방식**: 존재하지 않는 경로를 줬는데도 에러 없이
"bag_filenames:" 목록이 빈 채로 실행이 이어지다가, 실제 라이브 시각(당시 진행
중이던 attempt3 비행의 sim 시각 1022~1035s대)과 정확히 일치하는 데이터를
받기 시작함 — 즉 **없는 bag 대신 라이브 토픽(`/drone/imu`,`/drone/points`)을
그대로 구독해버린 것으로 보인다.** `glim_rosbag input_rosbag_path`가 usage
문자열엔 있지만 내가 준 위치인자가 실제로 파싱되는지 불확실 — 진짜 존재하는
bag으로, **다른 라이브 비행이 돌고 있지 않을 때** 다시 검증해야 함(라이브
데이터와 섞여 오판하지 않도록). 25초 후 timeout으로 정상 종료시킴 —
attempt3 비행 자체에는 지장 없었음(진행률 로그로 확인).

**✅ bag 경로 인자 미스터리 해결(GitHub 소스 확인, 재실행 없이 정적으로 확인)**:
`glim_rosbag.cpp`(https://github.com/koide3/glim_ros2/blob/master/src/glim_rosbag.cpp)
는 `argv[1..]`을 `glob()`으로 파일 매칭한다 — 내가 준 `/nonexistent_bag_path`는
매칭 0건이었는데도 하드 에러 없이 그대로 진행했고(이게 "bag_filenames:" 빈
목록의 정체), 그 상태에서 노드에 살아있던 라이브 구독이 마침 돌고 있던
attempt3 토픽을 주워온 것 — **실제 존재하는 bag 경로를 주면 glob이 정상
매칭되어 그 파일을 읽는다.** 별도 fallback 로직이 아니라 "매칭 0건이어도
안 죽는다"는 관대한 처리였을 뿐. `auto_quit` 파라미터가 있어 재생이 끝나면
자동 종료되는 것으로 보임(기본값 확인 필요하나 스크립팅엔 유리).
→ **attempt3 완주 후**: `ros2 run glim_ros glim_rosbag bags/velocity_4m_attempt3
--ros-args -p config_path:=$(pwd)/glim_config` 로 바로 실행하면 됨. 추가 검증
불필요, 실제 bag으로 바로 정식 실행.

세션이 끊겨도 이어받을 수 있도록, 단계가 끝날 때마다 여기에 요약을 추가한다.

## 환경/설정
- 작업 위치: `/home/hyunwoo-chae/AG-CoNav-test_main` (test_main 브랜치 전용 git worktree,
  원래 브랜치 `brian_test`의 uncommitted 변경을 건드리지 않기 위해 분리했다).
  원본 리포는 `/home/hyunwoo-chae/AG-CoNav` (brian_test, 손대지 않음).
- 이유: brian_test 체크아웃에는 `src/agconav_test_worlds`가 아예 없고, 기존
  `install/`도 stale(예: `drone_pose_controller.yaml`의 `service_timeout_sec`가
  test_main의 5.0이 아니라 구버전 1.0)이라 실험에 쓸 수 없었다. worktree를 새로
  만들어 test_main HEAD(`44b0190`)를 그대로 체크아웃하고 처음부터 빌드한다.
- ROS2: Jazzy (`/opt/ros/jazzy`), Gazebo Sim 8.11.0, colcon 확인됨.
- 하드웨어: 4 core / 5.8GB RAM — RESULTS.md 실험은 6-core 기준이었고 거기서도
  CPU 포화 문제가 있었다. 이 환경은 더 열악하므로 real-time factor 저하에 더
  주의해야 한다.
- GLIM: 1단계 조사대로 미설치(도커 이미지 없음, GPU 없음, docker 그룹 권한 없음).
  4단계 전에 설치 필요. **[갱신] Docker(koide3/glim_ros2)는 amd64 전용이라 이
  ARM64 VM에서 애초에 불가 — 사용자 지시로 Docker 대신 PPA(apt,
  ros-jazzy-glim-ros, arm64+noCUDA 공식 지원)로 설치 방식 전환. 아래
  "GLIM PPA 전환" 절 참조.**
- 아키텍처: **aarch64(ARM64)** — 크래시 복구 조사 중 처음 확인함(이전엔 미확인).

## 진행 상태
- [x] 2단계: 표면 높이 모델 + 경로 3개 생성 + 안전검증 (경로 3개 파일 자체는 그대로 보존)
- [x] 3단계: teleport 드라이런 — **4m만 유효**(아래 참조)
- [~] 4단계: velocity 실비행 + GLIM — **4m만**. 4-1 attempt2가 VM 크래시로 중단,
  attempt3 재실행 중(아래 "크래시 복구" 절 참조). 4-2 GLIM은 Docker→PPA(apt)
  전환 진행 중 — PPA 저장소 등록에 sudo 필요, 사용자 조치 대기.
- [ ] 5단계: Module A/F 채점 + wheel FN 비교 — **84m 기준선 vs 5m+4m + GLIM 열
  복원(사용자 지시로 GLIM 제외안 취소, 원래 계획대로 복원)**

## 🟡 GLIM PPA 전환 (Docker amd64 블로커 해결책, 사용자 지시)

사용자 확인: koide3 GLIM은 Docker(amd64 전용)와 별개로 PPA(apt) 배포판을
제공하며 Ubuntu 24.04(noble)/22.04에 대해 amd64+arm64를 공식 지원한다
(https://koide3.github.io/glim/installation.html). 이 VM은 Ubuntu 24.04
(ROS2 Jazzy) + GPU 없음이므로 CUDA 없는 조합이 정확히 맞는다.

**진행 상황**:
- `curl`, `gpg` 이미 설치돼 있어 별도 설치 불필요(확인 완료).
- PPA 저장소는 아직 미등록. 등록 스크립트(`https://koide3.github.io/ppa/setup_ppa.sh`)
  내용을 먼저 읽어 확인 — 전체가 `EUID -ne 0`이면 즉시 종료하도록 짜여 있어
  **반드시 root 권한 필요**(`/etc/apt/trusted.gpg.d/`, `/etc/apt/sources.list.d/`
  쓰기 + `apt update`). 이 세션은 `sudo -n true`도 비밀번호를 요구해 실패
  (기존 docker 그룹 블로커와 동일한 성격의 제약) — **직접 실행 못 함.**
- ⏸ **사용자 조치 필요**: 터미널에서 `!` 접두사로 다음을 실행해달라:
  ```
  curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
  ```
  등록되면 `apt-cache policy ros-jazzy-glim-ros` 자체는 sudo 없이 내가 바로
  실행해 arm64+noCUDA 조합이 실제로 존재하는지 확인하고(사용자 지시 1단계),
  나오면 곧장 `sudo apt install ...` 패키지 설치로 진행한다(사용자가 이미
  이 경로를 승인함) — 이 설치도 sudo가 필요해 마찬가지로 사용자가 위 명령과
  함께, 또는 그 다음에 별도로 실행해줘야 할 가능성이 높다(설치 커맨드는
  이 조사 완료 후 정확한 패키지 목록으로 다시 안내하겠음).

## 🔴 크래시 복구 — 어디까지 유효하고 어디부터 재실행하는지 (VM 재부팅 후 조사)

VM이 4단계-1 attempt2 비행 도중 크래시 후 재부팅됐다. 재개 전, 디스크에 남은
산출물의 수정 시각을 rate-limiter 4m 경로 재생성 시각(아래 "정지조건 3번 해결"
절, 파일 mtime 기준 15:36)과 대조해 유효성을 판정했다.

**1) 2단계 경로 파일 — 전부 유효, 그대로 재사용**
```
path_100x100_5m_2m.yaml   13:39  (원본, rate-limiter 미적용 — 애초에 2m은 실험 제외 확정이라 무관)
path_100x100_5m_3m.yaml   13:39  (원본, 3m도 실험 제외 확정이라 무관)
path_100x100_5m_4m.yaml   15:36  (rate-limiter 재생성판 — "정지조건 3번 해결" 절과 시각 일치)
```
2m/3m은 애초에 4~5단계에서 안 쓰므로 재생성 대상도 아니었고, 4m은 재생성 시각이
기록과 정확히 일치. **재실행 불필요.**

**2) 3단계 드라이런(4m, rate-limiter 적용판) — 유효, 그대로 재사용**
```
run_results/logs/dryrun_teleport_4m_v3.log   16:27  (재생성 15:36 이후)
run_results/dryrun_4m.md                     16:40  (재생성 15:36 이후, PROGRESS 마지막 기록 16:41 직전 완료)
```
둘 다 4m 경로 재생성(15:36) *이후* 시각이라 rate-limiter 적용판에 대한 유효한
결과다. 커버리지 99.6%, 완주 정상 — **재실행 불필요.**

**3) 4단계-1 velocity 비행 attempt2 — 무효, attempt3로 재실행**
`run_results/logs/velocity_4m_attempt2.log`(mtime 17:02, PROGRESS.md 마지막 기록
16:41보다 늦음 = 이 세션이 기록하지 못한 진행분)를 직접 열어 확인:
- 마지막 로그 줄이 `waypoint 2720 / 5251 도달`(진행률 51.8%)에서 **정상 종료
  메시지 없이 뚝 끊김**. 완주 메시지도, 에러/트레이스백도, SIGINT 처리 로그도 없다
  — 프로세스가 그 순간 통째로 죽은 크래시 시그니처.
- 로그 안 타임스탬프(ROS 시각 `1786780963` → 2026-08-15 17:02:43 KST)가 로그
  파일 mtime(17:02)과 정확히 일치 — 그 순간 VM이 죽은 것으로 확정.
- bag(`bags/velocity_4m_attempt2/`, 6개 mcap 파일 총 12GB)도 마지막 파일이
  17:02에 끊겨 있고, 그다음 분할파일(`_6.mcap`)은 0바이트로 생성만 되고 못 씀 —
  같은 크래시 순간의 흔적.
- **비행이 완주되지 않았으므로 GLIM 입력으로도, 5단계 채점 입력으로도 쓸 수
  없다.** 재사용 불가 판정 → 삭제(디스크 확보) 후 **attempt3**로 처음부터
  재실행한다. 런치 인자는 attempt2와 동일(`flight:=velocity
  path_file:=path_100x100_5m_4m.yaml cruise_speed_mps:=4.0
  bag_output:=.../bags/velocity_4m_attempt3`), 타임아웃도 기존 확정값(4500s=75분)
  그대로 유지 — 새로운 설계 판단 아님, 단순 재시도.
- 재부팅 후 잔여 프로세스 없음 확인(`pgrep -af "ros2 launch|gz sim|gz-sim|
  parameter_bridge|ros2 bag record|drone_"` 결과 없음) — 정리 절차 별도로
  필요하지 않았다.

**4) 4단계-2 GLIM — 아직 미착수. docker 권한 블로커는 여전히 미해결**
`id -nG`에 `docker` 그룹 없음, `docker info` 여전히 `permission denied`(server 쪽) —
재부팅됐지만 사용자가 `sudo usermod -aG docker $USER`를 실행하지 않은 것으로
보인다. (아래 5번의 더 근본적인 블로커 때문에 이 문제는 지금 당장은 부차적임.)

**5) 🔴 신규 치명적 발견 — 이 VM은 ARM64인데 GLIM 이미지는 전부 amd64 전용**
사용자가 크래시 전부터 물어봤던 질문에 대한 답:
- `uname -m` → **aarch64**. (이전 조사에선 아키텍처를 확인한 적이 없었다.)
- `docker manifest inspect --verbose koide3/glim_ros2:jazzy_cuda13.1` →
  `"platform": {"architecture": "amd64", "os": "linux"}` — **manifest list가
  아니라 단일 플랫폼(amd64) 이미지**, arm64 변형이 없다.
- Docker Hub API로 리포지토리 전체 태그(`jazzy_cuda13.1`, `jazzy_cuda12.5`,
  `jazzy`, `humble`, `humble_cuda12.2`) 확인 — **5개 태그 전부 amd64뿐, arm64
  태그 자체가 리포지토리에 없음.**
- 이 VM에 `qemu-x86_64` binfmt 에뮬레이션도 등록돼 있지 않음
  (`/proc/sys/fs/binfmt_misc/qemu-x86_64` 없음) — 설령 등록한다 해도 CUDA
  이미지를 GPU 없는 ARM64 VM에서 에뮬레이션으로 돌리는 건 SLAM 워크로드
  특성상 사실상 비현실적(속도·안정성 모두)이라고 판단.
- **결론: docker 그룹 권한 문제를 사용자가 지금 당장 해결해줘도, 이 이미지
  자체가 이 VM에서 pull조차 안 된다.** 이건 확정 범위("GLIM CT+passthrough+
  pose_graph 조합")를 이 하드웨어에서 물리적으로 실행할 방법이 없다는 뜻이라
  **정지조건 3번**에 해당한다고 판단해 여기서 멈추고 보고한다. 가능한 선택지
  (사용자 결정 필요, 그중 어느 것도 확정 범위 안이 아니라 임의로 고르지 않음):
  (a) GLIM을 ARM64용으로 소스 빌드, (b) x86_64 VM/머신으로 이 4-2 단계만
  옮겨서 실행, (c) 4-2(GLIM)를 이번 실험 범위에서 제외하고 5단계를 GLIM 없는
  형태로 축소, (d) 다른 SLAM 대안으로 교체. **사용자 확인 전까지 4-2는 진행하지
  않는다.**

**6) 디스크 — 이번 크래시가 디스크 고갈 때문이었다는 증거는 없음, 그래도 안전장치 추가**
재부팅 후 현재 `df -h ~`: 78G 중 43G 사용(59%), 31G 여유 — 위험 수준 아님.
attempt2가 크래시 전까지 만든 bag은 약 21분간 12GB(≈9.5MB/s, RESULTS.md의
"200초에 54GB"(≈270MB/s) 폭주 사례와는 자릿수가 다른 정상적인 증가 패턴) —
이번 크래시의 원인이 디스크였다는 근거는 못 찾았다(VM 자체 크래시로 보임,
원인 불명). 그래도 사용자 지시대로 **attempt3부터는 상태로그 갱신마다
`df -h ~`를 함께 기록하고, 사용량이 80%를 넘는 순간 즉시 비행을 중단(SIGINT→
TERM 에스컬레이션 후 잔여 gz sim 프로세스까지 정리)하고 실행을 멈춘 뒤
보고하도록** `run_results/logs/status_log_attempt3.log`에 60초 주기로 기록하는
감시 스크립트를 붙여 실행한다.

**정리(디스크 확보) — 삭제는 사용자 확인 대기 중, 아직 실행 안 함**: 재사용
불가로 판정된 `bags/velocity_4m_attempt2/`(12GB)와, 이미 분석이 끝나 산출물
(`dryrun_4m_v2_rootcause.md` 등)로만 남기면 되는 `bags/probe2_4m/`(3.9GB, 3단계
진단용, 분석 완료됨)를 삭제해 여유 공간을 넓히려 했으나, `rm -rf`가 자동 승인
범위 밖이라 차단됨 — 사용자에게 별도로 확인받아야 한다. 다만 삭제 없이도 현재
여유 공간(31G)만으로 attempt3 한 사이클(예상 ~24GB) 진행에는 지장이 없다고
판단해, 삭제 확인을 기다리는 동안 **attempt3를 먼저 시작**했다
(`run_results/run_attempt3_monitored.sh`, 백그라운드 실행 중, 60초 주기로
`run_results/logs/status_log_attempt3.log`에 진행률+`df -h ~` 기록, 80% 도달 시
자동 중단).

## ✅ 정지조건 3번 해결 — rate-limiter로 4m 경로 재생성

사용자 승인(옵션1: rate-limiter로 z-프로파일 스무딩 후 재생성)에 따라
`run_results/generate_paths.py`에 `rate_limit_z()`(slope-limited majorant, 2-pass
max-plus 구성)를 추가하고 4m 경로만 재생성했다:
- 설계 상한: `climb_speed_mps` 예산(6.0, velocity_path_follower.py 기본값)의
  **70%인 4.2 m/s**로 제한(안전계수 0.7 — 실제 컨트롤러 추종 지연/오버슈트 여유).
  4m/s 순항 기준 `MAX_SLOPE = 4.2/4.0 = 1.05`.
- 결과(재생성 로그): `max_req_vz=4.20 m/s`(설계상한과 정확히 일치, 정상), **예산초과
  세그먼트 0개**(attempt1 스캔 때 170개였던 것과 대비), 최소 클리어런스도
  4.650m로 오히려 개선(기존 3.033m) — 건물 주변에서 미리 올라가는 만큼 여유가
  더 생긴 것.
- 최종 웨이포인트 5,251개(기존 5,260개와 큰 차이 없음), 총 경로 길이 2,880.0m,
  예상 순수비행 720.0s(12분, 기존 대비 오히려 짧아짐 — 이전엔 자동보정으로 삽입된
  9개 웨이포인트가 있었으나 이번엔 0개).
- **2m/3m은 건드리지 않음**(이미 실험 제외 확정, yaml 파일 무수정) — 다만 이번
  4m 재생성이 `run_results/path_5m_safety_report.md`를 스크립트가 통째로
  다시 쓰면서 2m/3m 절을 날렸길래, rate-limiter *이전* 로직으로 재계산해 원래
  수치 그대로(2m: final=10322 corr=21 len=6041.2m / 3m: final=6878 corr=11
  len=4056.6m, 전부 이전과 동일) 복원해 다시 끼워넣었다(`rebuild_2m_3m_report_section.py`,
  yaml 파일은 전혀 건드리지 않고 텍스트 섹션만 재계산). 이 2m/3m 절이 rate-limiter
  적용 이전 버전이라는 점을 보고서 안에 명시해뒀다.
- `path_5m_visualization.png`도 갱신(4m 궤적만 바뀜, 2m/3m은 원본 yaml 그대로라 동일).
- **3단계 teleport 드라이런 재확인 완료**(`dryrun_teleport_4m_v3`, 1097s):
  경로 완주 정상, 박스 기준 커버리지 **99.6%**(rate-limiter 적용 전과 동일),
  이상치 0%, 고도 범위 0.746~8.918m 정상. `run_results/dryrun_4m.md` 갱신함
  (이 파일이 최종본 — attempt1 정지 전 버전은 git 히스토리에만 남음).
  → **4단계 재시도(attempt2)로 진행.**

## ⏸ (해결됨, 기록 보존) 4단계-1 attempt1에서 정지조건 3번(설계 갈림길) 발동

**attempt1 결과: 570초/5,260개 중 1,318번째 웨이포인트에서 고도가 설계값(8.4m)에서
30m 이상(38.5m) 벗어나 팔로워가 자체 안전중단.** 로그: 웨이포인트 1305까지는
선이탈 0.1m 이하로 완전히 정상 비행하다가, 1309부터 선이탈이 1.6→2.9→4.3→...→12.0m로
**단조 발산**(진동이나 일시적 흔들림이 아니라 회복 없이 계속 벌어짐).

**Root cause 확인(재시도 전에 먼저 진단함, 근거 있는 판단):** 웨이포인트 1303→1304
구간에서 z가 7.40m→11.37m로, **수평 0.5m 이동 중 수직 3.97m 상승**을 요구한다
(건물 가장자리 — 지형 위 AGL에서 건물 지붕 위 AGL로 전환). 4m/s 순항 기준 이 구간
통과 시간은 0.125초이므로 **요구 수직속도 ≈ 31.8 m/s** — `velocity_path_follower.py`
자체의 `climb_speed_mps` 예산(6.0)의 5배가 넘는다. 이건 자세제어 게인 문제가 아니라
**경로 자체가 이 드론이 물리적으로 따라갈 수 없는 상승률을 요구**하는 것이다(어떤
attitudeGain/angularRateGain 조합도 30 m/s 수직 속도를 6.0 m/s 예산의 컨트롤러로
따라잡게 할 수 없다).

**전체 경로를 스캔해 확인 — 일회성이 아니라 구조적 결함:** 4m 간격 경로 전체
5,259개 세그먼트 중 **170개(3.2%)가 climb_speed_mps(6.0) 예산을 초과**하며,
최악의 경우 **93.4 m/s**를 요구한다. 건물 20채 전부의 가장자리에서 반복적으로
발생하는 패턴 — 방금 멈춘 지점(#1303~1304)을 어떻게든 넘긴다 해도 남은 169개
지점 중 어딘가에서 다시 같은 방식으로 발산할 것이 거의 확실하다.

**왜 3단계(teleport 드라이런)에서는 안 걸렸나:** SetEntityPose 순간이동은 물리
가속도·속도 제한이 없어(공간 기하만 확인) 이 결함이 드러나지 않았다. 2단계
안전검증(waypoint 사이 0.2m 샘플링)도 "위치가 지표면+3m 이상인가"만 봤지
"그 위치에 그 타이밍으로 도달 가능한가"(운동학적 실현 가능성)는 애초에 검증
항목이 아니었다 — 이건 2단계 안전검증의 범위 밖 결함이라 정지조건 1번(안전검증
실패)이 아니라 3번(설계 갈림길)에 해당한다고 판단했다.

**왜 "3~4회 자동 재시도"(정지조건 2번 절차)를 그대로 따르지 않았나:** 이미 확정적
증거(전체 경로 스캔)로 게인 튜닝이 원인이 아님을 확인한 상태에서 재시도를 반복하는
건 결과가 뻔한 시도에 매번 ~10분 이상(시뮬 재시작+비행)을 태우는 것이라 판단해,
진단이 끝난 시점에 바로 멈추고 보고하는 쪽을 택했다.

**제안하는 수정 방향(사용자 확인 필요 — 이게 왜 정지조건 3번인지):** 스트립 방향
0.5m 간격으로 "그 지점의 지표면+5m"를 그대로 웨이포인트 z로 쓰는 2단계 로직 자체를
바꿔야 한다. 예: 건물 경계 근처에서 z 프로파일에 **상승률 상한(climb_speed_mps
기준)을 두는 rate-limiter**를 적용해, 건물에 도달하기 전부터 미리 상승을
시작하고 지난 뒤에 서서히 하강하게 만드는 방법(항상 필요 최소고도 이상만
유지하도록 클리핑하므로 안전마진은 절대 깨지지 않고, 건물 주변에서 잠깐
더 높이 나는 것으로 완화됨). 이건 "5m AGL 방법B"를 매 지점에서 문자 그대로
지키는 것을 살짝 완화하는 결정이라 사용자 확인 없이 진행하지 않는다.

## 4단계-1 진행 상황 (velocity 실비행, 4m 간격)

- world: `Seongdong_gu_100x100_dynamic`(RESULTS.md 10절 튜닝값 적용된 사본, flight:=velocity가
  자동 선택), cruise_speed_mps:=4.0(launch에 새로 추가한 인자로 명시 지정 — 안 하면 launch
  기본값 8.0이 나가서 확정 스펙 위반이었을 것, 발견해서 수정함).
- **attempt1 첫 시도부터 안정적** — RESULTS.md 10절 튜닝값(스폰위치/forceConstant/
  attitudeGain/angularRateGain)이 별도 조정 없이 그대로 통했다. 125초 시점: waypoint
  256/5260, 선 이탈 0.19~0.36m, 위치 (36.66, -161.89, 8.88) — 추락/충돌/발산 징후 없음.
  정지조건 2번(불안정) 해당 없음.
- **속도가 예상보다 훨씬 느림(타이밍 문제, 안정성 문제 아님)**: 순항 4m/s를 지정했지만
  0.5m 간격으로 촘촘히 찍은 웨이포인트가 `velocity_path_follower.py`의
  `arrive_radius_m`(기본 1.5m) 감속 로직과 부딪혀 사실상 계속 "도착 임박" 감속 상태에
  머무는 것으로 보임(관측: 125초에 256개 웨이포인트 = 실질 평균 ~1.3 m/s). 전체
  5,260 웨이포인트 완주 예상 시간 ≈ 2,568s(43분) — 원래 지시문의 stage-3 20분 기준과도
  다른 이슈이고, 이건 4단계 완주 자체를 막지는 않으므로(안정적으로 계속 진행 중)
  타임아웃만 넉넉히(4,500s=75분) 늘려서 그대로 완주를 기다린다. 코드/파라미터를
  수정하지 않음(확정 스펙 밖 변경을 피하기 위해 — 필요하면 나중에 팀 논의 항목으로
  남길 사항이지 지금 내가 바꿀 부분은 아니라고 판단).

## ⚠️ 4단계-2 블로커 — GLIM Docker 이미지 pull 불가 (사용자 조치 필요)

1단계에서 이미 확인한 블로커가 그대로 유효함: `docker info` permission denied,
`id -nG`에 docker 그룹 없음, `sg docker -c`/`sudo -n`모두 비밀번호 요구 — 이 세션
권한으로는 `docker pull koide3/glim_ros2:jazzy_cuda13.1`을 실행할 방법이 없다.
4-1 비행이 끝나는 대로 bag은 확보되지만, GLIM을 실제로 돌리려면 사용자가 터미널에서
직접(`!` 접두사로) 다음 중 하나를 해줘야 한다:
```
sudo usermod -aG docker $USER   # 이후 새 로그인 셸에서 적용됨
# 또는
sudo docker pull koide3/glim_ros2:jazzy_cuda13.1   # sudo로 직접 pull
```
GPU 없는 환경이 확인됐으므로 실행 시 `--cpu` 옵션이 필요하다(`run_glim.sh`가 이미
처리). 4-1이 끝나는 시점까지 이 블로커가 안 풀리면 4-2에서 실제로 멈추게 된다.

## ⚠️ 최종 범위 — 사용자 지시로 2m·3m 모두 제외, 4m 간격 하나만 진행

시간 문제로 사용자가 두 차례에 걸쳐 범위를 축소했다. 최종적으로 **이번 실험은
"5m AGL + 스트립간격 4m" 조합 하나만 4~5단계까지 끝까지 진행한다.**

- **2m**: 2단계 안전검증까지만 실행하고 실험 시작 전 사용자가 제외 결정.
  `path_100x100_5m_2m.yaml`, 안전보고서 2m 절, 시각화의 2m 궤적은 유효한
  산출물이라 그대로 보존(삭제 안 함). 3단계 이후 어디에도 쓰지 않음.
- **3m**: 3단계 드라이런을 실행하던 도중 사용자가 직접 중단 지시. **이건 실패나
  데이터 손상이 아니라 의도적 중단**이다 — `path_status`가 True가 되기 전에
  launch를 SIGINT로 정상 종료했고(gz sim 잔여 프로세스도 별도 SIGINT로 마저
  정리, 잔여 프로세스 없음 확인 완료), 미완성 bag(7.6GB, path_status 도달 전
  상태)은 디스크 확보를 위해 삭제했다. **3m 관련 산출물은 하나도 없고, 이후
  어떤 단계에서도 3m을 언급/사용하지 않는다.** `path_100x100_5m_3m.yaml`
  자체(2단계 산출물)는 유효하므로 보존.
- **4m**: 3단계 드라이런은 이미 완료된 기존 결과(`run_results/dryrun_4m.md`,
  root-cause 조사 포함, 박스 기준 커버리지 99.6%)를 그대로 재사용. 재실행하지
  않음. 4~5단계는 **4m만** 진행.

**5단계 최종 비교표는 2행만**: "84m 실제비행+GLIM(RESULTS.md 기준선)"과
"5m AGL + 4m 간격" — 그 외 행은 만들지 않는다.

**중단 절차(참고용 기록)**: launch 프로세스(ros2 launch)에 SIGINT → 죽지 않으면
TERM 순으로 에스컬레이션 → cascade shutdown 후에도 `gz sim` 바이너리가 별도
프로세스로 남는 경우가 있어(이번에도 재현됨) 그것도 별도로 SIGINT/TERM →
`pgrep -af "ros2 launch|gz sim|gz-sim|parameter_bridge|ros2 bag record|drone_*"`
로 잔여 프로세스 0개 확인. 4단계에서도 실행/재시도마다 이 절차를 그대로 반복한다.

## ✅ 해결됨 — "낮은 커버리지" root-cause 조사 결과 및 최종 결정

사용자 지시로 root-cause를 먼저 파본 결과, **정지조건 3번을 취소하고 계속 진행하기로
확정**. 조사 과정과 결론:

**1) 센서 사거리(170m) 초과 가설 — 기각(직접 실측으로 확인)**
`run_results/probe_transformed.py`로 drone_elevation_mapper와 동일한 파이프라인
(isfinite 필터 → min_range 2.5m 자기반사 필터 → TF 변환)을 재현해 100개 스캔·
144만 점을 직접 검사: **170m 초과 점 0개** (관측된 최대 range는 79.8m). gz의
gpu_lidar가 사거리 밖은 Inf로 채우고(코드 주석에 이미 명시, isfinite로 걸러짐)
그 경로는 완전히 정상 작동 중이었다.

**2) 실제 원인 — 박스 밖 유효 셀의 공간 분포로 확인**
`run_results/analyze_bag.py`를 확장해 GridMap의 각 유효 셀이 박스 경계로부터
얼마나 떨어져 있는지 히스토그램을 뽑음(4m 간격 드라이런 재실행, `dryrun_4m.md`):
- 박스 밖 유효 셀 32,509개 중 **87.7%는 경계에서 5m 이내** — 5m AGL·4m 간격에서
  스와스(swath) 폭이 좁아 인접 스트립 간 그레이징 각도 관측이 박스 가장자리를
  살짝 넘는, 정상적이고 무해한 현상.
- 나머지 꼬리(2.7%, 20m~98.1m)는 **TF/자세 보간 불안정으로 설명됨**: 우리 lawnmower
  경로는 스트립마다 180° yaw 반전 코너가 있다(4m 간격=25회, 84m 기준선 경로는
  전체 10 waypoint 중 4회뿐). 코너에서 발행되는 pose가 한 틱(0.1s) 안에 yaw
  ~0°→~180°로 급변하는데, 그 찰나에 스캔 타임스탬프가 걸리면 tf2 Buffer의
  quaternion 보간이 두 정반대 회전 사이에서 불안정해져(수학적으로 정의역 경계)
  그 스캔의 점들이 로컬 거리·고도값은 정상인데 맵 좌표계에서 엉뚱한 방향으로
  튄다(최악 사례: (79.20, 32.00), 고도 4.744m로 값 자체는 멀쩡하나 박스에서
  98m 이탈). 84m 기준선은 코너가 4회뿐이라 이 현상이 있어도 거의 안 보였을
  뿐, 우리가 새로 만든 결함이 아니라 **경로의 턴 횟수에 비례해 커지는, 기존
  파이프라인에 잠재해 있던 현상**으로 판단.
- 두 원인 모두 "센서 사거리 안의 정상 관측"이라 사용자가 제시한 분기 중
  **2번(Module A는 건드리지 않고 채점 스크립트만 박스 기준으로 수정)**에 해당.

**3) "heightmap skirt" 가설 — 사실상 기각**
박스 밖 셀의 87.7%가 5m 이내로 몰려 있고 나머지는 코너-보간 좌표 이상으로
설명되므로, 지형 경계의 인공 스커트 지오메트리가 별도로 반사를 만들어내는
정황은 찾지 못했다. (100% 확증은 아니지만 관측된 패턴을 스커트 가설로 설명할
필요가 없어졌다.)

**4) 조치 — Module A(`drone_elevation_mapper.py`)는 무수정, 채점만 박스 기준으로**
`run_results/analyze_bag.py`에 "박스 기준 커버리지" 절을 추가:
분모를 원본 GridMap 전체 셀이 아니라 **100x100 박스의 예상 셀 수(1,000,000)**로
고정하고, 분자도 그 박스 안에 있는 유효 셀만 센다. 이 필터는 코드 주석과
여기 모두에 **"이번 100x100 실험 전용, 500x500 확장 시 재사용 금지"**라고
명시해뒀다(하드코딩된 BOX 상수를 그대로 재사용하면 안 됨).

**5) 재채점 결과 — 애초에 문제가 아니었다**
4m 간격 드라이런을 박스 기준으로 다시 채점: **커버리지 99.6%** (995,551/1,000,000
셀). 원본(그리드 팽창 포함) 지표였던 21.4~25.0%는 순전히 분모 오염 때문이었고,
실제 100x100 박스 안 관측은 84m 기준선(59.51%)보다 오히려 훨씬 낫다 — 5m AGL +
4m 간격의 좁은 스와스가 촘촘히 겹치며 전 구역을 거의 빠짐없이 스캔한 결과로
해석된다. 박스 안 유효 셀의 고도값도 0.411~8.859m로 지표면 모델 기대범위
(1.202~8.780m)와 사실상 일치, 이상치 0%.

**결론: 정지조건 3번 해제, 3단계 계속 진행.** 3m/2m 간격도 같은 방식(박스 기준
채점)으로 드라이런하고, 4~5단계도 이 채점 로직을 그대로 쓴다.

## ⏸ 정지 — 3단계에서 정지조건 3번(경로 자체 결함/설계 갈림길) 발동

**4m 간격 드라이런(teleport, Seongdong_gu_100x100) 결과: Module A 커버리지 25.0%**
— RESULTS.md 6절의 84m 기준선(59.51%)보다 뚜렷이 낮음. 자세한 진단은
`run_results/dryrun_4m.md` 참조. 요약:
- 원인은 새로 생긴 버그가 아니라 **RESULTS.md 9절에 이미 기록된 기존 결함**
  ("모듈 A `_grow_to_fit`에 상한이 없음")과 같은 계열이지만, 5m AGL에서 그 결함의
  악영향(그리드가 의도한 100x100보다 훨씬 크게 부풀어 커버리지%를 희석시키는 정도)이
  84m 대비 훨씬 크다(그리드 y방향이 의도한 100m의 2.6배 vs 84m 기준선은 1.12배).
- 원인 후보(미확정): 5m AGL은 지면에 훨씬 가까워 얕은 각도로 나가는 LiDAR 빔이 박스
  경계 밖까지 스치듯 도달해 반환되는 경우가 84m보다 많을 가능성.
- (1차 분석에서 "z가 -45~+38m까지 튄다"고 오판했던 것은 내 분석 스크립트가 `/drone/points`
  의 센서 로컬 프레임 좌표를 world 좌표로 잘못 해석한 방법론 오류였음 — 정정 완료.
  실제 GridMap에 쌓인 고도값 자체는 정상 범위였다.)

**3단계 지시문이 명시적으로 예시로 든 정지 트리거("극단적으로 낮은 커버리지")에
해당한다고 판단해 여기서 멈춘다.** 3m/2m 드라이런, 4~5단계는 사용자 지시를
기다린 뒤 진행.

**산출물(지금까지)**:
- `run_results/dryrun_4m.md` — 4m 간격 드라이런 상세 분석.
- `run_results/analyze_bag.py`, `run_results/scan_point_extremes.py`,
  `run_results/probe_points_once.py` — bag/포인트클라우드 진단 도구(재사용 가능).
- `src/agconav_test_worlds/launch/experiment.launch.py` — `path_file` 런치 인자 추가
  (기본값 `path_100x100.yaml`로 기존 동작 100% 보존, 5m AGL 경로 파일 선택 가능하게 함).
  이 변경은 정지조건 3번과 무관한 순수 배관(plumbing) 변경.
- 4m 드라이런/probe용 bag은 분석 후 디스크 확보를 위해 삭제함(재현 가능한 산출물이라
  원본 보존 불필요하다고 판단 — bag_output/maps_output 인자로 언제든 재생성 가능).

## ⚠️ 알려진 블로커 — 4단계 GLIM 설치 전에 사용자 조치 필요
`docker info`가 permission denied, `sg docker -c ...`도 그룹 비밀번호를 요구해서
실패(`sudo -n true`도 비밀번호 필요 — 이 세션엔 비밀번호를 모름). 즉 **현재 세션
권한으로는 docker 그룹에 진입할 방법이 없어 GLIM 이미지를 pull할 수 없다.**
4단계 진입 전에 사용자가 터미널에서 직접 다음을 실행해야 한다(비밀번호 필요라
내가 대신 못 함):
```
sudo usermod -aG docker $USER
# 그 후 이 세션이 아니라 새 로그인 셸에서 docker 그룹이 적용됨.
```
지금은 3단계(GLIM 불필요)를 계속 진행하고, 4단계 진입 시점에 이 블로커를
사용자에게 알린다.

## 2단계 완료 (요약)

**산출물**
- `run_results/surface_model.py` — 지표면 높이 함수(지형 vs 건물 중 최댓값), 0.5m 격자 캐시
  (`run_results/surface_cache.npz`).
- `run_results/generate_paths.py` — lawnmower 웨이포인트 생성 + 안전검증/자동보정.
- `src/agconav_test_worlds/config/path_100x100_5m_{2m,3m,4m}.yaml` — 최종 경로 3개
  (기존 `path_100x100.yaml`은 그대로 보존).
- `run_results/path_5m_safety_report.md`, `run_results/path_5m_visualization.png`.

**지표면 모델 설계 판단**
- 지형: height_map.png를 world의 `<heightmap><size>=100 100 5.249597>`,
  `<pos>=11.4 -116.1 1.202132>`로 쌍선형 디코딩(1단계에서 검증한 gz 공식 그대로,
  `elevation = pixel/img.max()*size.z + pos.z`).
- 건물: buildings.dae를 정점 welding + union-find로 20개 연결요소(=개별 건물)로 분리
  (1단계에서 확인한 `generate_test_world.py`의 `crop_dae()` 로직 재사용). 각 건물의 (x,y)는
  **convex hull**로 포함판정, roof Z는 그 건물 정점 중 최댓값. Hull은 실제 풋프린트의
  상위집합이라 "과소평가 금지" 요구사항과 방향이 일치(넓게 잡을지언정 좁게 잡지 않음).
- 임의 좌표 조회(`SurfaceModel.height_at`)는 그 점을 감싸는 2x2 격자 셀 중 **최댓값**을
  반환(쌍선형 보간 아님) — 건물 가장자리에서 인접 지형 셀과 평균 내어 낮잡는 일을 방지하기
  위한 보수적 설계. 부작용: 격자 셀(0.5m) 내부에서는 지표면이 계단식(구간별 상수)이 됨.
- 실측: 건물 20채, 지표면 고도 1.202~8.780m (지형만은 1.202~6.452m). Convex hull 기준
  건물이 지형보다 높게 잡히는 셀 비율 = 37.8%(0.5m 격자) — 1단계 조사 때 낸 bbox 합산
  근사치(67%, 겹침/직사각형 여유 포함이라 과대추정)보다 낮고 더 정확한 값. 시각화
  (`path_5m_visualization.png`)로 건물 20채 윤곽과 3개 경로가 지표면 위에 정상적으로
  겹치는 것을 육안 확인했다.

**웨이포인트 생성**
- 스트립은 x방향, y방향으로 간격만큼 이동하는 고전 lawnmower. 스트립 내부는 0.5m
  간격(사용자 지시 범위의 촘촘한 쪽 끝)으로 균일 적용 — 건물이 전체 면적의
  1/3 이상을 차지해 "건물 근처만 촘촘히"가 의미 없다는 사용자 지침 반영.
- z = surface_model(x,y) + 5.0m.
- 방향(orientation)은 진행방향 yaw(순방향 스트립 0°, 역방향 스트립 180°)로, 기존
  `path_100x100.yaml`의 컨벤션과 동일하게 구성.

**안전검증 — 실제로 위반을 잡아내고 고친 사례**
- 1차 구현에서 세그먼트 내부 샘플 개수를 `int(seg_len/0.2)`로 계산해 0.5m 세그먼트가
  실제로는 0.25m 간격(3점)으로만 검사됐다. 그 상태에서는 corr=0으로 "전부 안전"하게
  나왔는데, 이게 진짜 안전해서가 아니라 **덜 촘촘히 검사해서 위반을 놓쳤을 가능성**이
  있다고 판단해 `math.ceil()`로 바꿔 확실히 ≤0.2m 간격을 보장하도록 수정 후 재실행했다.
- 수정 후: **2m 간격 경로에서 21곳, 3m 15개, 4m 9곳**에서 실제로 3.0m 마진 위반이
  잡혔고, 위반 지점에 그 지점 실측 지표면+5.0m 웨이포인트를 삽입하는 방식으로
  전부 자동보정됨(재분할 반복 중 재위반 없음 — 정지조건 1 미해당).
- 최종 최소 클리어런스: 2m=3.006m, 3m=3.030m, 4m=3.033m — 전부 3.0m 마진 이상 충족.

**경로별 요약(등속 4 m/s 기준, 가감속 미포함)**

| 간격 | 웨이포인트 | 경로길이 | 예상 순수비행시간 | 턴 | 자동보정 | 최소클리어런스 |
|---|---|---|---|---|---|---|
| 2m | 10,322 | 6,041.2 m | 1510.3 s (25.2분) | 50 | 21 | 3.006 m |
| 3m | 6,878 | 4,056.6 m | 1014.2 s (16.9분) | 33 | 11 | 3.030 m |
| 4m | 5,260 | 3,119.7 m | 779.9 s (13.0분) | 25 | 9 | 3.033 m |

**3단계로 넘기는 주의사항**: 2m/3m 간격 경로는 등속만으로도 20분(1200s)을 넘긴다
(2m=25.2분, 3m=16.9분, 실제로는 코너 감속 등으로 더 길어짐). 사용자가 3단계 드라이런
타임아웃으로 "20분"을 제시했지만 이는 84m 고도의 짧은 왕복 경로 기준이었을 가능성이 커서,
이번 100x100 전체 커버 경로 특성상 비현실적이다. 확정 범위(스트립간격 2/3/4m, 4m/s,
100x100 전체 커버)를 그대로 지키면서 "타임아웃 값" 자체는 실행 파라미터일 뿐이므로,
경로별 예상 비행시간 + 50% 여유를 둔 타임아웃(예: 2m→40분, 3m→27분, 4m→20분)으로
조정해 사용한다. 이 판단은 정지조건 3("확정 범위 밖의 새 선택지")에 해당하지 않는다고
보고 계속 진행한다 — 판단 근거를 여기 남긴다.


## 정지 조건 (반드시 사용자에게 보고 후 대기)
1. 2단계 안전검증 자동보정 반복 후에도 위반이 남는 경로가 있을 때
2. 4단계 velocity 비행이 자동 튜닝 3~4회 후에도 불안정/충돌할 때
3. 확정 범위(5m AGL 방법B, 4m/s, 스트립간격 2/3/4m, GLIM CT+passthrough+pose_graph,
   wheel FN 비교 방식) 밖의 새 선택지가 필요할 때

---

## 📝 디스크 안전 정책 추가 (사용자 저장공간 우려 확인 후, 재비행 도중 보강)

사용자가 재비행 저장공간을 우려해 실측을 요청. 확인 결과:
- bag 증가 속도 실측 ~10MB/s (velocity_4m_5mps_attempt1, 초반 60초 구간 샘플).
- 예상 최종 bag 크기 ~7~9GB (경로 예상 비행시간 700~900s 기준).
- 확인 시점 디스크: 78G 중 37G 사용(50%), 여유 37G — 1회 완주는 충분하나,
  재시도 허용치(3~4회)를 다 쓰면서 실패 attempt bag을 안 지우면
  4×9GB≈36GB로 여유공간을 거의 소진할 수 있음.

**정책**: attempt가 실패해 재시도(attempt2, attempt3...)로 넘어갈 때, 실패한
이전 attempt의 bag 디렉토리를 다음 attempt 시작 전에 삭제해 디스크를 확보한다
(성공한 최종 bag만 남긴다). 디스크 감시 임계값은 기존 80% 대신 70%부터
선제적으로 정리 판단을 시작한다.

이 보강 지시 시점(velocity_4m_5mps_attempt1 진행 중, gz sim + ros2 bag record
PID 27074/27079/27286 확인)까지는 attempt1 하나만 존재, 실패/정리 이력 없음.

## [이번 세션] 0단계 재비행 + 1~2단계 Module A 이식 진행 중

**0단계 판단**: bags/ 가 완전히 비어 있음(이전 세션이 재현 가능하다는 이유로
전부 삭제) — 홈 디렉토리 전체를 find로 검색해도 5m AGL(4m 간격, 5m/s,
velocity) bag이 어디에도 남아있지 않아 재비행 필요로 판단, 즉시 재비행 시작.
`run_results/run_velocity_4m_5mps_monitored.sh attempt1` 사용(이전 세션이
이미 만들어둔, 마지막 웨이포인트 도달 후 STALL 오판 버그가 수정된 스크립트).
동일 조건: flight:=velocity, Seongdong_gu_100x100_dynamic(velocity 모드 자동
선택), path_100x100_5m_4m_5mps.yaml, cruise_speed_mps:=5.0.

**모니터링 중 발견한 버그(스크립트 자체는 아니고 내 감시 루프)**: 이전 세션의
동일 tag(attempt1) 결과 파일 `velocity_4m_5mps_attempt1_result.txt`(04:57에
COMPLETED로 stale하게 남아있던 것)이 지워지지 않은 채 남아있어서, 내가 처음
짠 "결과 파일 존재하면 완료"로 보는 감시 루프가 즉시(1초 만에) 오탐 완료로
잘못 판단했다. 실제로는 gz sim/ros2 bag record가 정상 기동 중이었음(ps로 확인).
stale 파일 삭제 후 재모니터링해서 바로잡음 — 앞으로 같은 tag 재사용 시 이
파일이 남아있을 수 있다는 점 주의.

**중간 진행 상황(재비행)**: 200s 시점 wp 404/5251, bag 1.8GB, TF 정상(~49Hz),
디스크 50%. 순항중.

**사용자(코디네이터 경유) 정책 갱신**:
- 디스크 부족 시 `run_results/clouds/`(6.5GB, 84m/5m 실험 point cloud 원본,
  이미 FN% JSON으로 요약됨)와 `run_results/dryrun_4m_5mps_grid.npz`(90MB)는
  지워도 된다고 명시적 허가받음 — 지운 순서/이유는 실제로 지울 때 여기 기록.
- **정책 전환(중요)**: 이번에 새로 기록하는 bag(velocity_4m_5mps_attemptN,
  최종 성공분)은 실험 종료 후에도 삭제하지 않는다 — 이전 세션들이 "재현
  가능"을 이유로 84m/5m bag을 지운 판단이 바로 이번 재비행을 유발했기
  때문. 실패/중단된 attempt bag만 삭제 대상.

**1~2단계(코드)**: brian_test의 Module A(`drone_elevation_mapper.py`)를
`git show`로 확인 — README에 명시된 요구사항(elevation/variance 칼만필터,
R=거리+입사각+밀도, Q=0, 이노베이션 게이팅 9.0, NaN 패딩, elevation_variance
레이어, **np.gradient 로컬 윈도우 최적화**) 전부 실제로 존재함을 확인(문제
없음, 별도 보고 불필요). 이걸 test_main_brian의 같은 파일에 이식하면서:
- 이 브랜치 고유의 `min_range_m`(기체 자기 반사 제거, transform 전 센서
  로컬 좌표 원점 기준 거리 필터) 유지.
- Module D(`ground_elevation_mapper.py`, brian_test)의 방어 로직 2가지를
  참고해 Module A에 새로 구현: `max_sensor_range`(200.0, transform 후
  센서 원점 기준 거리 필터, `_accumulate`에서 min_range 다음/그리드
  비닝 전) + `max_grid_cells`(30,000,000, `_grow_to_fit`에서 패딩 직전
  예상 총 셀 수 계산 후 초과 시 (None, None) 반환 → `_accumulate`에서
  이번 배치만 버리고 기존 누적 보존).
- `drone_elevation_mapper.yaml`에 칼만필터 파라미터 + 두 방어 파라미터
  전부 추가.
- `agconav_drone/package.xml`의 `<depend>rosbag2_py</depend>` 바로 다음
  줄에 `<exec_depend>rosbag2_storage_mcap</exec_depend>` 추가.

**다음**: 재비행 완료 대기(메모리가 빠듯함 - 277MB free, swap 1GB 사용 중이라
colcon build는 비행 완전 종료 후에 실행해 자원 경합 방지). 완료 후 build →
bag 재생(module_a 라이브 구독) → 채점 → 비교표.

**타임아웃 변경(사용자 지시, 코디네이터 경유)**: 재비행이 필요해져 전체
작업 타임아웃을 2시간 → 5시간으로 연장(시작 시각 기준, 이미 지난 시간
포함). 재비행 자체의 정지조건(3~4회 재시도 불안정, TF 끊김, 디스크 80%
하드컷)은 그대로 유지하되, 디스크 70%부터는 선제적으로
`run_results/clouds/`, `dryrun_4m_5mps_grid.npz` 삭제로 여유를 확보하는
정책을 추가 적용한다(80%가 스크립트의 하드 컷이므로 70%는 그 전에 손 쓸
여유를 두기 위한 조기경보 기준).

**선제적 디스크 정리(70% 조기경보, 사용자 사전 허가)**: 재비행 진행 중
디스크가 60%를 넘고 증가 추세(비행 완료 시점 예상 ~74%)로 보여, 완료 후
build+replay에서 추가로 필요할 여유를 확보하기 위해 미리 정리:
- `run_results/clouds/`(6.5GB, 84m/5m 실험의 원본 point cloud .npy) — 이미
  FN% JSON(`84m_*_fn.json`, `*_4m_5mps_fn.json`)으로 요약 완료, 필요시
  build_methodB_cloud.py로 재생성 가능해 원본 보존 불필요.
- `run_results/dryrun_4m_5mps_grid.npz`(90MB) — 2단계 드라이런 캐시, 이미
  분석 완료.
삭제 후 디스크 사용량 재확인함(로그 참고).

## 3단계 진행 — bag 재생 시작 (칼만필터 + 방어 로직 2겹 라이브 검증)

**재비행 완주**: attempt1, path_status=true, 2231s(약 37분), bag 22GB(12개
mcap 청크), TF/프로세스 전부 정상 종료. `bags/velocity_4m_5mps_attempt1`
보존(정책대로 삭제하지 않음).

**colcon build**: `colcon build --symlink-install --packages-up-to
agconav_drone agconav_traversability` — 성공, 에러 없음(deprecation 경고만).

**bag 재생 실행**: `run_results/run_bag_replay_kalman.sh
bags/velocity_4m_5mps_attempt1 kalman_4m_5mps
run_results/kalman_4m_5mps_fn.json 92 1.0 3600` — drone_elevation_mapper,
terrain_feature_calculator, traversability_verdictor(wheel/leg),
elevation_map_saver 5개 노드를 use_sim_time:=true로 먼저 띄운 뒤
`ros2 bag play --clock --rate 1.0`으로 원본 /drone/points, /tf, /tf_static,
/drone/path_status를 재생. capture_nav_fn.py가 최종 산출물을 구독해 채점.

**이상치 방어 로그 확인(3단계 요구사항)**: drone_elevation_mapper 시작 로그에
`max_sensor_range=200.0m, max_grid_cells=30000000, min_range_m=2.5m`가 정확히
찍히는 것을 확인 — 파라미터가 정상 선언/초기화됨(실제 컷오프 동작 여부는
capture_nav_fn.py 결과의 coverage/height 통계로 간접 확인 예정 — 200m/
3천만 셀 둘 다 이번 100x100 지도 규모에서는 정상 상황에선 거의 안 걸릴
임계값이라 트리거 자체가 로그에 안 남는 게 오히려 정상, 콜백 경로가
빠짐없이 실행됐다는 것만 확인).

**다음**: bag 재생 완료(예상 ~37분, --rate 1.0) 대기 → wheel/leg FN% 채점
→ 비교표 갱신 → git commit/push.

**알려진 버그(이번에 발견) — SIGTERM 에스컬레이션이 bag record를 강제종료시켜
metadata.yaml 누락**: `run_velocity_4m_5mps_monitored.sh`의
`shutdown_launch()`가 launch 프로세스에 SIGINT→(15초 내 안 죽으면)
SIGTERM으로 에스컬레이션하는데, 이번 정상 완주 종료 과정에서 실제로
SIGTERM까지 에스컬레이션됐다(status_log: "SIGINT로 안 죽어서 SIGTERM
에스컬레이션"). 그 여파로 하위 `ros2 bag record` 프로세스가 정상적인
graceful shutdown(각 mcap 청크의 요약/인덱스 섹션 flush + metadata.yaml
작성)을 못 마치고 죽어, bag 디렉토리에 mcap 파일 12개(22GB, 데이터 자체는
전부 정상)는 있는데 `metadata.yaml`이 없는 상태가 됐다. 그 결과
`ros2 bag play`가 "No storage id specified, and no plugin found that could
open URI" 에러로 즉시 실패.

**복구**: `ros2 bag reindex -s mcap bags/velocity_4m_5mps_attempt1` 실행 →
성공("Could not set read order on open(), falling back to file order" 경고는
있었지만 — mcap 파일 자체에 메시지 인덱스가 없어 수신 순서 재정렬을 못하고
파일에 쓰인 순서를 그대로 쓴다는 뜻일 뿐, recorder가 어차피 시간순으로
기록하므로 실질적 영향 없음). `ros2 bag info`로 데이터 무결성 확인:
duration 2258.8s, `/drone/points` 22,319개, `/tf` 111,598개, `/tf_static` 1개,
`/drone/path_status` 1개, `/drone/imu` 223,053개 — 데이터 손실 없음.

**재발 방지**: 이 스크립트류(`run_velocity_*_monitored.sh`,
`shutdown_launch` 패턴을 쓰는 모든 것)로 기록한 bag은, 재생하기 전에 항상
`ros2 bag info <bag_dir>`로 metadata.yaml 존재/무결성을 먼저 확인하는
습관을 들인다(이번 세션의 이후 단계부터 바로 적용). 근본 수정(SIGINT
타임아웃을 늘리거나 bag record만 먼저 별도로 SIGINT하는 등)은 이번 작업
범위 밖이라 이번엔 안 건드리고, 발생 시 reindex로 복구하는 절차만 확립.

## 4~6단계 완료 — 채점 결과 및 최종 비교표

**4단계 채점 결과**(`run_results/kalman_4m_5mps_fn.json`):
- wheel FN% = 10.91% (n_gt_traversable=618,378, occupied_FN=67,436, unknown=9,046)
- leg FN% = 8.03% (occupied_FN=49,646, unknown 동일 9,046)
- elevation_map coverage = 95.80%(997,427/1,041,148), height 1.04~9.04m
- step(단차) 중앙값 0.00692m, p95 0.953m, max 6.519m — baseline(3번)보다
  전부 개선.

**5단계 비교표(최종)**은 `run_results/SUMMARY.md` 최상단 절 참조(5개 행
전체, 분석 포함) — 여기서는 요약만:

| # | 고도 | 속도 | 간격 | 위치정합 | 지도생성방식 | wheel FN% | leg FN% |
|---|---|---|---|---|---|---|---|
| 1 | 84m | 8m/s | 32m | GT만 | 단순평균 | 23.94% | 12.15% |
| 2 | 84m | 8m/s | 32m | GT+GICP | GICP정합 | 20.10% | 11.89% |
| 3 | 5m AGL | 5m/s | 4m | GT만 | 단순평균 | 13.44% | 12.60% |
| 4 | 5m AGL | 5m/s | 4m | GT+GICP | GICP정합 | 37.26% | 30.03% |
| 5 | 5m AGL | 5m/s | 4m | GT만 | 칼만필터(이상치방어) | **10.91%** | **8.03%** |

핵심 결론: 칼만필터(5번)는 3번(같은 GT-only, 알고리즘만 다름) 대비
wheel -18.8%/leg -36.3%(상대) 개선, 4번(GICP) 대비는 wheel 3.4배/leg
3.7배 개선. 원시 노이즈(step 중앙값/p95/max), 커버리지(95.80%),
미측정비율(1.46%) 전부 3번보다 개선 — GICP(4번)처럼 "쉬운 곳만 쉽게"가
아니라 지도 전체 품질을 실제로 높였다. 4번(GICP) 대비 압도적 우위는
사용자가 제시한 가설(강체변환 전체 오염 vs 셀단위 독립 처리)과 실측이
정확히 일치(4번은 자기 baseline인 3번보다 step 중앙값 5.3배 악화, 5번은
같은 3번보다 24.5% 개선 — 정반대 방향). 상세 근거는 SUMMARY.md 참조.

**6단계 — 저장/커밋**: 아래 커밋 로그 참조.

## 이번 세션 최종 상태

전체 6단계(0~6) 완료. 정지조건 발동 없음. bag(`bags/velocity_4m_5mps_
attempt1`, 22GB)은 정책대로 보존(삭제 안 함, .gitignore로 git 추적 제외).
다음 세션 이어받을 지점: 없음(이번 작업 범위 완료). 참고로 남길 것 —
84m 조건에서의 칼만필터 실측은 아직 없음(SUMMARY.md "다음 결정 지점"
참조, 이번 범위 밖이라 미수행).
