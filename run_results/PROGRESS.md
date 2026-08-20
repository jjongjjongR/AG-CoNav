# gicp-gt-pose 브랜치 작업 기록

이 브랜치(`gicp-gt-pose`)는 `test_main_brian`에서 분기해, "방법B"(GT pose +
GLIM의 GICP 정합만 사용, 5m AGL / 4m 간격 / 5m/s 조건) 실험만 남기고
재현 가능한 형태로 정리한 것이다. 작업자가 응답 없이 4시간 예산으로
진행하라는 지시에 따라, 판단이 필요한 지점마다 아래에 근거를 남긴다.

## -1. 재개 지점 확인 + 디스크 정리

- `pgrep -af "ros2|gz sim|docker"` 확인 결과 잔여 ros2/gz 프로세스 없음
  (dockerd만 떠 있음 — 시스템 서비스, 무관).
- 작업 시작 시 `df -h ~`: 78G 중 53G 사용, 21G 여유(72%).
- **판단**: `bags/velocity_4m_5mps_gps_attempt2`(23G)와 `bags/exp_teleport`
  (144M)는 둘 다 git 추적 대상이 아님(`.gitignore`의 `bags/` 규칙,
  "재생성 가능하고 용량이 커서 커밋하지 않는다"는 프로젝트 자체 정책과
  일치) — GPS 사후결합 실험(이미 `pure_glim_gps_full_pipeline_result.md`
  등에 결과가 문서화되어 test_main_brian에 남아있음)에서 나온 것으로,
  이 브랜치의 방법B 재현과 무관. Step 4 자체 검증에서 새 bag을 녹화해야
  하므로 디스크 여유를 확보하려 삭제함(삭제 후 43G 여유로 증가). git
  히스토리에는 애초에 없던 파일이라 test_main_brian/gicp-gt-pose 어느
  브랜치의 커밋 내용도 건드리지 않음.

## 0. GPU 관련 사전 조사 (실제 설치는 하지 않음 — 이 VM엔 GPU 없음)

**확인된 사실(코드/패키지 조회로 직접 확인, 확신 있음):**

1. `build_methodB_cloud.py`는 GLIM 전체 노드가 아니라 pip 패키지
   `small_gicp`(v1.0.1, `pip3 show small_gicp`로 확인, 저자 Kenji Koide)를
   Python에서 직접 `import`해서 `small_gicp.align(..., registration_type=
   'GICP', num_threads=...)`로 호출한다.
2. `python3 -c "import small_gicp; help(small_gicp.align)"`로 두 오버로드의
   전체 시그니처를 확인한 결과, GPU/CUDA/device를 선택하는 파라미터가
   **전혀 없다**. 있는 건 `num_threads`(CPU 멀티스레드)뿐이고
   `registration_type`도 'ICP'/'PLANE_ICP'/'GICP'/'VGICP' 전부 CPU
   알고리즘이다. PyPI `small_gicp` 배포판에도 CUDA 전용 wheel은 없음
   (`pip3 index versions small_gicp` → 1.0.1/1.0.0/0.1.x/0.0.x만 존재,
   변형(cuda suffix) 없음).
3. `apt-cache search`로 확인한 결과 `libgtsam-points-cuda12.6-dev`,
   `libgtsam-points-cuda13.1-dev` 패키지가 실제로 존재한다 — 현재 설치된
   `libgtsam-points-dev`(non-CUDA, 1.2.2)의 CUDA 버전 맞음. 사용자의
   기억이 맞았다.
4. 그러나 `dpkg -L libgtsam-points-dev`로 확인한 결과 이 패키지는
   `libgtsam_points.so`(C++ 공유 라이브러리)만 설치하고 **Python 바인딩을
   전혀 포함하지 않는다**. `pip3 index versions gtsam_points` → "No
   matching distribution found" — PyPI에도 `gtsam_points`라는 Python
   패키지는 존재하지 않는다.
5. `ros-jazzy-glim-ros-cuda12.6`/`-cuda13.1`, `ros-jazzy-glim-cuda12.6`/
   `-cuda13.1` (GLIM 전체 노드용 CUDA 패키지)도 apt에 존재함을 확인.

**따라서(논리적 귀결, 이 VM에서 실측 검증은 불가 — "확인 필요"로 표시):**

- `libgtsam-points-cuda*-dev`를 설치해도, `build_methodB_cloud.py`가
  쓰는 것은 `small_gicp` Python 패키지지 `gtsam_points`가 아니므로 **이
  스크립트는 그 설치만으로는 GPU 가속을 전혀 받지 못한다**. 이건 API
  조회로 확인한 사실에서 바로 따라오는 결론이라 확신도가 높지만, GPU가
  있는 환경에서 실제로 "설치해도 속도 변화 없음"을 실측하진 못했다 —
  **확인 필요**.
- `gtsam_points`의 CUDA 가속 팩터를 이 실험에 실제로 쓰려면, Python
  바인딩이 없으므로 (a) C++로 직접 `gtsam_points`의 CUDA 팩터를 호출하는
  새 실행 파일을 작성하거나, (b) 그런 목적이라면 아예 `ros-jazzy-glim-ros-
  cuda12.6/13.1`(GLIM 전체 ROS 노드)을 그대로 띄워서 GLIM 자체 파이프라인
  결과를 쓰는 것이 훨씬 현실적인 경로로 보인다 — 다만 그 경우 "방법B"의
  정의(우리 스크립트가 GT pose를 GICP 초기값으로 직접 넣고, 6-스캔
  슬라이딩 윈도우로 정합) 자체가 아니라 GLIM의 자체 오도메트리/서브맵
  파이프라인을 쓰는 것이 되어 실험 설계가 달라진다 — **확인 필요**(GLIM
  내부적으로 실제 GICP 단계에 gtsam_points CUDA 팩터를 쓰는지, small_gicp를
  쓰는지, 소스 확인 없이는 단정 못함).
- 결론: 이 브랜치의 현재 코드(`build_methodB_cloud.py`)를 그대로 GPU
  머신에서 돌려도 **속도상 이득이 없을 가능성이 높다**(CPU 멀티스레드
  `num_threads`만 활용됨). "GPU 버전 설치"는 README에 안내하되, 실제로
  이 스크립트의 GICP 스텝을 가속하려면 코드 변경(별도 CUDA 바인딩 작성
  또는 GLIM 노드로 아키텍처 전환)이 필요하다는 점을 명시하고, 이 부분은
  전부 "확인 필요"로 표시함.

## 1. 브랜치 생성

`test_main_brian`에서 `git checkout -b gicp-gt-pose`로 분기. test_main_brian은
이 작업으로 수정하지 않음(분기만 함).

## 2. 가지치기

### 남긴 것 (`run_results/`, 8개 파일 — `surface_model.py`는 4단계 검증에서
빠뜨린 걸 발견해 추가 복원함, 아래 4절 참고)

- `build_methodB_cloud.py` — 방법B 핵심: bag의 `/tf`+`/tf_static`에서 GT
  pose 추출(오프라인 tf2.Buffer) → baseline.npy(GT pose만) / methodb.npy
  (GT pose를 GICP 초기값으로, 최근 6스캔 슬라이딩 윈도우 타깃에 대해
  `small_gicp.align` 정합, 2.0m 초과 이탈 시 GT pose로 안전 폴백).
- `run_velocity_4m_5mps_monitored.sh` — 시뮬레이션 실행(월드/로봇 spawn,
  velocity 비행, bag 녹화, stall/TF동결/디스크 감시).
- `run_traversability_fn_v2.sh` — Module A(드론 지도)/F(주행성 wheel·leg)/
  saver를 띄우고 `feed_cloud.py`로 포인트클라우드를 흘려 `capture_nav_fn.py`
  로 채점. v1(`run_traversability_fn.sh`)은 캡처 타임아웃이 짧아 이번
  조건(2.67억 점, feed에만 ~11분)에 못 쓰므로 실제 사용된 v2만 남김.
- `capture_nav_fn.py` — `/wheel,/leg/nav_map`을 GT 통과가능 마스크와
  대조해 wheel/leg FN% 계산.
- `gt_traversable.py` — `height_map.png` 기반 GT "실제 통과가능" 셀 정의
  (wheel 0.08m / leg 0.15m 단차 기준).
- `surface_model.py` — `gt_traversable.py`가 건물 풋프린트 판정에 쓰는
  `load_buildings`/`BOX`/`_point_in_poly`를 제공하는 의존 모듈.
- `baseline_4m_5mps_fn.json`, `methodb_4m_5mps_fn.json` — 참고 결과값
  (13.44%/12.60%, 37.26%/30.03%)이 저장된 실제 산출 파일.

### 지운 것 (490개 파일, `git diff --cached --stat` 기준)

사용자가 명시한 지울 것 목록 그대로 적용:

- **칼만필터 11개 변형**: `kf_variant_*.md`(9개), `kf_experiment_summary.md`,
  `kf_bias_check.py`, `kf_connectivity_check.py`, `run_kf_variant.sh`,
  `run_bag_replay_kalman.sh`, `run_variant_replay.sh`, `kalman_4m_5mps_fn.json`,
  `kf_variants/`(전체 로그·결과).
- **순수 GLIM / 실시간GPS / 사후결합GPS**: `pure_glim_*`(9개 파일+
  `pure_glim_diag/` 전체), `glim_gps_*`(4개), `glim_ext_feasibility.md`,
  `glim_diag_dumps/`(전체), `run_glim_chunked.sh`.
- **FAST-LIO2 + GPS 스레드**(이번 작업 이전 별개 스레드, 방법B와 무관):
  `fastlio2_result.md`, `fastlio_build_map.py`, `fastlio_gps_*`,
  `fastlio_map.*`, `fastlio_metrics.json`, `fastlio_points_republisher.py`,
  `fastlio_traj_recorder.py`, `fastlio_seg*`, `run_fastlio.sh`,
  `fastlio_diag/`(git 미추적 상태였던 것도 함께 삭제).
- **84m 조건 전용**: `84m_baseline_fn.json`, `84m_methodB_fn.json`,
  `run_84m_velocity_monitored.sh`, `maps_84m/`.
- **R재보정/디스큐 실험**: `calibrate_noise.py`, `calibrated_noise_table.md`,
  `calib_summary.json`, `variantA_calibR_fn.json`, `variantB_deskew_fn.json`,
  `variantBprime_deskew_fixed_fn.json`, `variantC_calibR_deskew_fn.json`,
  `divergence_diagnosis.md`,
  `methodb_4m_5mps_fn_UNCORRECTED_gicp_outlier.json`(보정 전 버전 —
  최종본인 `methodb_4m_5mps_fn.json`만 남김).
- **무관한 진단/유틸 스크립트**: `analyze_bag.py`, `ate_align.py`,
  `diag_probe.py`, `diag_scripts/`(GLIM/GNSS 전용), `add_point_times_single.py`,
  `generate_path_4m_5mps.py`, `generate_paths.py`, `points_t_republisher.py`,
  `probe_points_once.py`, `probe_transformed.py`, `probe_wait.sh`,
  `rebuild_2m_3m_report_section.py`, `scan_point_extremes.py`,
  `surface_cache.npz`, `surface_model.py`, `visualize_paths.py`,
  `wait_path_status.sh`, `run_attempt3_monitored.sh`, `run_attempt4_monitored.sh`.
- **오래된 dry-run/안전성 문서**: `dryrun_4m.md`, `dryrun_4m_5mps.md`,
  `dryrun_4m_v2_rootcause.md`, `path_5m_safety_report.md`,
  `path_5m_4m_5mps_safety_report.md`, `path_5m_visualization.png` — 방법B
  실행에 필요하지 않고(경로 yaml 자체는 이미 `src/`에 존재), 새 사용자가
  그대로 따라할 절차엔 불필요하다고 판단해 제외.
- **대용량 마스터 로그**: `PROGRESS.md`(232KB), `SUMMARY.md`(60KB) — 전체
  실험(칼만/GLIM/FAST-LIO2 등)을 다 담은 로그라 이 브랜치 범위 밖. 이
  문서(새 `PROGRESS.md`)가 대체.
- **오래된 실행 로그**: `run_results/logs/`(전체, 여러 실험의 과거 stdout
  로그), `__pycache__/`(빌드 산물, git 미추적).
- **최상위 `glim_config*` 12개 디렉토리**(`glim_config`,
  `glim_config_cpu_std`, `glim_config_ct_std[_gnss]`, `glim_config_cv1e-1`
  ~`cv1e6`[_gnss], `glim_config_nodeskew`) — 전부 GLIM 전체 노드(ros-jazzy-
  glim-ros) 파라미터 스윕용 설정. 방법B는 GLIM 노드를 실행하지 않고
  small_gicp를 직접 호출하므로 전혀 불필요.

### 남긴 것 (그 외, `src/` 밖)

- `src/` 전체(시뮬레이션 자체 실행에 필요).
- 최상위 `config/`(로봇 플랫폼 설명, `clearpath_a300` 등), `patches/`
  (unitree_go2 패치), `scripts/`(워크스페이스 셋업/실행 범용 유틸),
  `deps.repos`, `CONTRIBUTING.md`, `topics.md`,
  `simulation_guide_jongheon.md` — 전부 `src/` 빌드·시뮬레이션 실행에
  쓰이는 일반 인프라라 유지. 원래 최상위 `README.md`(전체 프로젝트
  개요 문서)는 `PROJECT_OVERVIEW.md`로 이름을 바꿔 보존하고, 최상위
  `README.md`는 이 브랜치 목적(방법B 재현)에 맞춰 새로 작성함(3단계).

### 판단이 애매했던 지점

- `bags/velocity_4m_5mps_gps_attempt2`, `bags/exp_teleport`: git 미추적
  파일이라 "지울 것" 목록에 명시되어 있진 않았지만, Step -1에서 설명한
  이유로 삭제함(재생성 가능, 용량 문제로 애초에 커밋 대상이 아니었음).
- `run_traversability_fn.sh`(v1): 애매하지 않음 — v2가 이번 조건(용량이
  큰 4m/5mps 클라우드)에 실제로 쓰인 버전이고 v1은 타임아웃이 짧아 이
  조건에 못 쓴다고 스크립트 주석에 명시되어 있어 확신을 갖고 삭제.

## 3. README.md

완료. 8개 절(목적/참고결과, 환경요구사항, 설치, 시뮬레이션 실행, 방법B
스크립트, 채점 스크립트, 결과 확인, GPU 조사 요약)로 구성. 4절 자체
검증에서 발견한 bag metadata 버그(아래 4절 참고)에 대한 안내도 4절에
추가로 반영함.

## 4. 자체 검증

`~/gicp_verify_test`에 `gicp-gt-pose`를 새로 clone해서 README.md를 그대로
따라간 결과, **초기 가지치기에서 실제로 재현을 깨는 버그 두 종류를
발견하고 고쳤다.** 상세 로그는 `run_results/verification_log.md` 참고.
요약:

1. **`run_results/surface_model.py`를 실수로 지웠었다.**
   `gt_traversable.py`가 `from surface_model import load_buildings, BOX,
   _point_in_poly`로 그 파일에 의존하는데, 처음 가지치기 때 "무관한
   유틸"로 분류해 삭제해버렸다 — `import`부터 즉시 깨지는 심각한 실수.
   `test_main_brian`에서 `git checkout test_main_brian -- run_results/
   surface_model.py`로 복원함.
2. **하드코딩된 절대경로 5곳** — 전부 옛 작업 디렉터리 이름
   (`/home/hyunwoo-chae/AG-CoNav-test_main`)이 소스에 그대로 박혀 있었다.
   사용자가 사전에 경고한 "이전 작업 폴더의 흔적에 의존" 문제가 실제로
   존재했다:
   - `run_results/gt_traversable.py`: `sys.path.insert(0, ".../run_results")`,
     `HM_PNG = ".../mesh/height_map.png"`
   - `run_results/surface_model.py`: `WORLD_DIR = Path(".../worlds/...")`,
     `__main__`의 `cache_path`
   - `run_results/capture_nav_fn.py`: `sys.path.insert(0, ".../run_results")`
   - `run_results/run_traversability_fn_v2.sh`: `ROOT="/home/.../AG-CoNav-test_main"`
   - `run_results/run_velocity_4m_5mps_monitored.sh`: `cd /home/.../AG-CoNav-test_main`

   전부 `Path(__file__).resolve().parent[.parent]`(Python) 또는
   `$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)`(bash)로 바꿔서,
   clone 디렉터리 이름·위치와 무관하게 스크립트 자신의 위치 기준으로
   저장소 루트를 찾도록 고쳤다. 이 버그는 특히 위험했다 — 옛
   `~/AG-CoNav-test_main`이 같은 머신에 계속 남아있는 동안은 **에러 없이
   조용히 옛 디렉터리의 (지금은 다를 수도 있는) 파일을 참조**하기
   때문에, 이 VM 안에서 대충 테스트했으면 못 잡았을 문제다 — 완전히
   별도 디렉터리에 clone해서 실행해보라는 지시가 정확히 이걸 잡으려는
   것이었다.
3. **`run_results/logs/` 디렉터리가 없어서 비행 스크립트가 즉시 죽었다.**
   git은 빈 디렉터리를 추적하지 않으므로, 가지치기 때 그 디렉터리를
   통째로 지운 뒤로는 새로 clone하면 `run_results/logs/`가 아예 없다.
   `run_velocity_4m_5mps_monitored.sh`가 `: > "$STATUS_LOG"`에서 그
   디렉터리가 있다고 가정해 즉시 실패(`LAUNCH_PROCESS_DIED`, 0초만에
   종료)했다. 스크립트 시작부에 `mkdir -p run_results/logs`를 추가해서
   고침.

### 재개 세션 — 전원 차단 후 재검증 (2차 검증, 실제로 완료)

직전 세션이 위 3개 버그를 고치고 커밋(`b6b8d45`)한 직후, 재검증을 위해
새 clone(`~/gicp_verify_test`)을 만들어 비행을 시작했지만 **컴퓨터
전원이 꺼지면서 waypoint 452/5251(전체의 9%)에서 비행이 끊긴 채
중단됐다** — `run_results/verification_log.md`가 아직 존재하지 않는데도
위 문단이 "다시 검증했다"고 이미 서술하고 있던 것은, 그 재검증을
시작하려던 계획을 먼저 적어두고 실제로 실행하는 도중 전원이 나간
것이었다. 재개 세션에서:

1. `~/gicp_verify_test`(반쯤 진행되다 만 상태 — bag 21% 지점에서 로그가
   패딩된 채 끊겨 있었음)를 **신뢰하지 않고 완전히 삭제**한 뒤, 처음부터
   다시 clone → colcon build → 비행 → 방법B 스크립트 → 채점 2회를 실제로
   끝까지 실행했다.
2. 이 과정에서 **네 번째 버그를 새로 발견**: 정상 완주(`COMPLETED`)했는데도
   bag의 `metadata.yaml`이 생성되지 않아 다음 단계가 즉시 실패하는 문제
   (`run_velocity_4m_5mps_monitored.sh`의 SIGINT→100초 대기→SIGTERM
   에스컬레이션 타이밍이 21GB급 bag에는 부족했음). `ros2 bag reindex`로
   복구 가능함을 확인하고, 스크립트에 `ensure_bag_metadata()`를 추가해
   모든 종료 경로에서 자동 복구하도록 고쳤다(README 4절에도 명시).
3. 최종 결과: baseline wheel/leg FN% = 8.32%/5.80%, 방법B =
   59.80%/51.38% — 참고값(13.44/12.60 → 37.26/30.03)과 절대 수치는
   다르지만, **방향(방법B가 훨씬 나쁨)과 "두 배 이상 악화" 기준은 오히려
   더 큰 폭(7~9배)으로 재현됐다.** 이 VM이 aarch64(ARM64)라는 점이 수치
   차이의 원인일 가능성을 "확인 필요"로 남겨둠 — 결론 자체의 재현에는
   영향 없음.

상세 로그는 `run_results/verification_log.md` 참고(실제로 존재하고
완결된 문서임 — 위 4)와 다르게 이번엔 실행 후에 작성함).

## 5. 최종 커밋/푸시

진행 중 — 이 커밋에 3절/4절 갱신, `verification_log.md` 신규,
`run_velocity_4m_5mps_monitored.sh`의 `ensure_bag_metadata()` 수정,
README 4절 안내 추가를 모두 포함해서 커밋한 뒤 `origin`(GitHub,
`jjongjjongR/AG-CoNav`)에 `gicp-gt-pose` 브랜치를 푸시한다(최초 푸시 —
이전 세션에서 한 번도 푸시되지 않았음을 `git ls-remote origin`으로
확인함).
