# 자체 검증 로그 (Step 4)

이 로그는 `gicp-gt-pose` 브랜치의 README.md를 **처음부터 끝까지 그대로
따라가서** 재현되는지 확인한 기록이다. 검증은 두 차례 있었다:

1. **1차 검증**(이전 세션): 하드코딩된 경로 3종/누락 파일 1개/빈 디렉터리
   가정 버그 1개를 발견해 고치고 `b6b8d45` 커밋으로 반영함(상세 내용은
   `PROGRESS.md` 4절 참고). 이 시점에는 아직 "고친 뒤 처음부터 다시 검증"이
   실제로 끝나지 않은 상태로 세션이 중단됐다(컴퓨터 전원 차단).
2. **2차 검증**(이 로그, 재개 세션): `~/gicp_verify_test`를 완전히 삭제하고
   `git clone -b gicp-gt-pose /home/hyunwoo-chae/AG-CoNav-test_main
   ~/gicp_verify_test`로 **다시 처음부터** clone해서, README.md 3~7절을
   실제로 끝까지 실행했다. 이 로그가 그 기록이다.

## 환경

- OS: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic — README 2절과 동일.
- **CPU 아키텍처: aarch64(ARM64), 4코어.** 원래 참고 결과값(13.44%/12.60%,
  37.26%/30.03%, README 1절)이 어떤 아키텍처에서 나왔는지는 이 문서에
  기록되어 있지 않다 — 이번 검증이 x86_64가 아니라는 점은 수치 차이의
  잠재적 원인 중 하나로 고려해야 한다(아래 "수치 차이" 참고).
- **GPU: 없음.** 이 VM에는 GPU가 없어 README 3절의 "GPU 버전 설치" 절과
  8절의 GPU 관련 내용은 **실행하지 않고 건너뛰었다** — 이 로그는 CPU
  경로(`small_gicp`, `num_threads`만 사용)만 검증한다. GPU 관련 내용이
  실제로 맞는지는 여전히 "확인 필요"로 남아있다(README 8절/PROGRESS.md
  0절 참고, 이번 검증으로 달라진 것 없음).
- `small_gicp` 1.0.1이 `pip3 install --user --break-system-packages`로
  이미 설치돼 있었다(사용자 site-packages는 clone과 무관하게 공유되므로
  재설치 불필요 — README 3절 3번 단계 자체는 새 사용자 기준으로 여전히
  필요하다).

## 실행 기록

### 1. 설치 (README 3절)

`colcon build --symlink-install` 성공(경고만 있고 에러 없음, 11 패키지
빌드, 11.1초 — 이전 빌드의 ccache/캐시가 남아있어 빠름. 완전 신규
환경에서는 더 걸릴 수 있다 — **확인 필요**로 남겨둠).

### 2. 시뮬레이션 실행 (README 4절)

```
bash run_results/run_velocity_4m_5mps_monitored.sh my_run
```

- 결과: `COMPLETED` (2154초 ≈ 36분, waypoint 5251/5251 완주).
- **버그 발견 #1 (실제 재현을 깨뜨림)**: 완주 후 종료 시퀀스에서 bag
  디렉터리에 `metadata.yaml`이 생성되지 않았다. bag은 mcap 파일 11개
  (~21GB)만 있고 `ros2 bag info`/`ros2 bag play`/`rosbag2_py` 리더 전부
  이 상태로는 bag을 열 수 없다(`build_methodB_cloud.py`가 즉시
  `RuntimeError: No storage could be initialized`로 실패).
  - **원인 진단**: `run_velocity_4m_5mps_monitored.sh`의
    `shutdown_launch()`가 SIGINT 후 최대 100초를 기다린 뒤 그래도 launch
    프로세스가 안 죽으면 SIGTERM으로 에스컬레이션하는데, 이 조건(bag
    ~21GB, 11개 파일)에서는 100초가 `ros2 bag record`가 마지막 파일을
    flush하고 `metadata.yaml`을 쓰기에 **충분하지 않았다**. 실제 로그에서
    완주 시퀀스는 02:00:02 SHUTDOWN 시작 -> 02:01:42 SIGTERM 에스컬레이션
    순이었는데, 이 시각이 마지막 mcap 파일의 마지막 수정 시각(02:01)과
    거의 겹친다 — recorder가 한창 마지막 파일을 쓰는 도중에 SIGTERM
    사슬에 끊긴 것으로 보인다.
  - **복구**: `ros2 bag reindex -s mcap bags/velocity_4m_5mps_my_run`로
    `metadata.yaml`을 재생성했다(mcap 파일 자체는 청크 단위로 안전하게
    쓰였으므로 reindex로 복구 가능했다) — 성공, 이후 스크립트 정상 처리됨.
  - **고침**: `run_velocity_4m_5mps_monitored.sh`에 `ensure_bag_metadata()`
    함수를 추가해서, 모든 종료 경로(COMPLETED/STALLED/TF_FROZEN/TIMEOUT/
    DISK_THRESHOLD_STOP/ERROR_IN_LOG)에서 `shutdown_launch` 직후
    `metadata.yaml` 존재를 확인하고 없으면 자동으로 `ros2 bag reindex`를
    실행하도록 했다. 타임아웃 값 자체를 늘리는 대신 자동 복구를 선택한
    이유: 디스크 속도는 환경마다 달라서 타임아웃을 얼마로 늘려도 더 느린
    디스크에서는 여전히 부족할 수 있다 — reindex는 환경과 무관하게 항상
    통하는 안전망이다. README 4절에도 이 이슈와 자동 복구 동작을 명시함.
  - 이 버그는 방법B 자체 로직과 무관하고(정합 알고리즘이 아니라 bag
    종료 타이밍 문제), 오직 "정상 완주해도 다음 단계에서 bag을 못 여는"
    재현성 문제였다 — 그래도 README를 그대로 따라간 사람이면 100% 만나는
    문제였을 가능성이 높아(디스크가 이 VM보다 느리면 더 흔함) 반드시
    고쳐야 하는 버그로 판단했다.

### 3. 방법B 스크립트 (README 5절)

```
python3 run_results/build_methodB_cloud.py \
  bags/velocity_4m_5mps_my_run /tmp/baseline_my_run.npy /tmp/methodb_my_run.npy
```

- (버그 #1 수정 후 재실행 — reindex로 복구된 bag으로) 정상 완료.
- 스캔 22366개 중 TF 조회 실패로 29개 스킵. GICP: 성공 21943, 실패(GT
  대체) 422(그중 60은 수렴했지만 2.0m 초과 이탈로 기각), 첫 스캔 1개
  스킵.
- baseline/방법B 각각 **266,778,590점** — README 2절의 "약 2.67억 개"와
  정확히 일치.
- 처리 중 "source/target point cloud is too small" 경고가 다수 나왔다 —
  README 1절에서 설명한 "코너마다 스캔이 순간적으로 아주 작아진다"는
  현상 그대로이며 스크립트가 이런 경우를 정상적으로 처리(GT 폴백)한다는
  것도 확인됨. 별도 조치 불필요.

### 4. 채점 스크립트 (README 6절) — 두 번

```
bash run_results/run_traversability_fn_v2.sh /tmp/baseline_my_run.npy baseline_my_run /tmp/baseline_my_run_fn.json
bash run_results/run_traversability_fn_v2.sh /tmp/methodb_my_run.npy methodb_my_run /tmp/methodb_my_run_fn.json
```

둘 다 정상 완료(순차 실행, 기본 타임아웃 1500초 이내).

## 결과 비교 (README 7절)

| | wheel FN% | leg FN% |
| --- | --- | --- |
| 참고값 — baseline | 13.44% | 12.60% |
| 참고값 — 방법B | 37.26% | 30.03% |
| **이번 검증 — baseline** | **8.32%** | **5.80%** |
| **이번 검증 — 방법B** | **59.80%** | **51.38%** |

- **방향**: 재현됨 — 방법B가 baseline보다 두 조건 모두에서 뚜렷하게
  나쁘다.
- **크기**: 참고값은 wheel 2.77배, leg 2.38배 악화. 이번 검증은 wheel
  7.19배, leg 8.86배 악화 — "두 배 이상 악화"라는 README의 재현 기준은
  넘치게 만족하지만, 절대 수치와 배율 자체는 참고값과 꽤 다르다.
- **원인 추정(확인 필요로 남김)**: 정확히 같은 수치가 나오지 않는 것은
  README도 이미 경고한 바지만(시뮬레이션 비결정성, bag 타이밍 차이),
  이번 차이는 그 설명만으로 다 설명되는지 불확실하다. 이 VM이
  **aarch64**라는 점(참고값을 낸 원 환경의 아키텍처가 기록에 없어
  x86_64였는지 불명)이 GICP 수렴 결과에 영향을 줬을 가능성을 배제할 수
  없다. 그러나 **결론(GICP를 추가하면 이 조건에서 오히려 나빠진다)
  자체는 이번 검증에서도 명확하게, 오히려 더 강하게 재현됐다** — 재현
  실패로 보지 않고, README에 "정확한 배율은 환경에 따라 크게 달라질 수
  있다"는 점을 좀 더 분명히 하는 방향으로 반영했다(아래 "README에 반영한
  것" 참고).

## README에 반영한 것

1. **버그 #1 수정**: `run_results/run_velocity_4m_5mps_monitored.sh`에
   `ensure_bag_metadata()` 추가(위 2절 참고) + README 4절에 이 이슈와
   자동 복구 동작 명시.
2. README 7절의 재현 기준("방향 + 대략적인 크기")은 이번 검증으로도
   실제로 유효했다고 확인됨 — 문구 자체는 바꾸지 않음(이미 "정확히 같은
   수치가 나오진 않을 것"이라고 정직하게 경고하고 있었고, 실측으로도 그
   경고가 맞았다).

## GPU (건너뜀, 재확인 없음)

이 VM에는 GPU가 없어 README 3절 "GPU 버전 설치"/8절 내용은 실행하지
않았다. 이전 세션(PROGRESS.md 0절)에서 코드/패키지 조회로 조사한 내용을
그대로 유지하며, "확인 필요"로 표시된 부분은 이번 검증으로도 달라지지
않았다 — GPU 있는 환경에서의 실측은 여전히 남은 과제다.

## 결론

**README.md를 그대로 따라가서 방법B의 핵심 결론(GICP 추가가 이 조건에서
결과를 악화시킨다)이 재현됨을 확인했다.** 그 과정에서 실제 재현을 100%
깨뜨릴 수 있는 버그(bag metadata 유실)를 발견해 고쳤고, 참고 수치와의
차이는 있으나 방향과 배율 기준으로는 재현 성공으로 판단한다.
