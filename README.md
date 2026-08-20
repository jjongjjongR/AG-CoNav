# gicp-gt-pose — 방법B(GT pose + GICP 정합) 재현 실험

이 브랜치는 AG-CoNav 시뮬레이션에서 나온 한 가지 실험만 다룬다: **드론이
낮은 고도(5m AGL)에서 촘촘한 간격(4m)으로 빠르게(5m/s) 스캔했을 때, GT
pose에 GLIM의 GICP 정합(`small_gicp`)을 추가로 걸면 지형 지도 품질이
좋아지는가, 나빠지는가?**

이미 한 번 이 조건에서 돌려서 **GICP가 오히려 결과를 악화시킨다**는
결과를 얻었다(아래 7절 참고). 이 문서는 그 결과를 **재현/검증**하기 위한
것이다. 전체 AG-CoNav 프로젝트(3로봇 통합 시뮬레이션) 자체에 대한 설명은
[`PROJECT_OVERVIEW.md`](PROJECT_OVERVIEW.md)를 참고하라 — 이 문서는 그
위에서 이 실험 하나를 처음부터 끝까지 그대로 따라 칠 수 있게 하는 것이
목적이다.

> 칼만필터·실시간/사후결합 GPS·FAST-LIO2·84m 조건 등 이 저장소에서
> 나왔던 다른 실험들은 이 브랜치에 없다. 그건 `test_main_brian` 브랜치에
> 있다. 이 브랜치는 방법B(5m AGL/4m/5mps) 하나만 남기려고 의도적으로
> 가지치기했다 — 무엇을 왜 지웠는지는 [`run_results/PROGRESS.md`](run_results/PROGRESS.md)에
> 기록되어 있다.

---

## 1. 무엇을 테스트하는가

- **월드**: 100×100m 성동구 동적 월드(`Seongdong_gu_100x100_dynamic`).
- **비행**: 고도(AGL) 5m, 스캔 라인 간격 4m, 순항 속도 5m/s
  (`path_100x100_5m_4m_5mps.yaml`). 낮은 고도·촘촘한 간격·빠른 속도라
  코너(라운마워 경로의 180도 턴)가 25개나 있고, 코너마다 스캔이 순간적으로
  아주 작아진다 — 이게 아래 결과 해석의 핵심이다.
- **비교 대상 두 가지**(둘 다 드론의 실제 물리 pose, 즉 GT pose를 씀 —
  SLAM 추정 pose 아님):
  - **baseline**: GT pose로 스캔을 그대로 지도에 쌓는다. 정합 없음.
  - **방법B**: GT pose를 초기 정렬값으로 주고, 직전 6개 스캔을 타깃으로
    GICP(`small_gicp`, GLIM이 쓰는 것과 같은 정합 라이브러리)로 미세
    정합한 pose로 쌓는다. GICP가 수렴 안 하거나(또는 수렴은 했지만 GT 대비
    2.0m 넘게 이탈하는, 물리적으로 말이 안 되는 해면) GT pose로 안전
    폴백한다.
- **채점**: 드론 2.5D 지도 → wheel(0.08m 단차 기준)/leg(0.15m 단차 기준)
  주행가능 맵으로 변환한 뒤, `height_map.png` 기반 GT "실제 통과가능"
  셀과 대조해 **FN%**(실제로는 통과 가능한데 파이프라인이 "막힘"으로
  오판한 비율)를 wheel/leg 각각 계산한다.

### 우리가 이미 얻은 참고 결과값 (이걸 재현/검증하는 것이 이 문서의 목적)

| | wheel FN% | leg FN% |
| --- | --- | --- |
| baseline (GT pose, 정합 없음) | **13.44%** | **12.60%** |
| 방법B (GT pose + GICP) | **37.26%** | **30.03%** |

**즉 이 조건에서는 GICP를 추가하는 게 오히려 FN%를 2배 이상 악화시켰다.**
원인으로 진단된 것: 코너마다 스캔이 아주 작아지는 구간에서 GICP가
"수렴은 했지만" 물리적으로 말이 안 되는 국소해(예: z가 튀는 등)로 잘못
정합되는 사례가 실제로 관측됐다(`build_methodB_cloud.py` 상단 주석
참고). 이 문서를 그대로 따라가서 **같은 방향의 결과(방법B가 baseline보다
나쁨)가 재현되는지, 정확한 수치가 위 표와 비슷한지**를 확인하는 것이
검증의 핵심이다.

---

## 2. 환경 요구사항

| 항목 | 값 |
| --- | --- |
| OS | Ubuntu 24.04 LTS (Noble) |
| ROS | ROS 2 **Jazzy Jalisco** |
| 시뮬레이터 | Gazebo Harmonic (gz-sim 8) |
| Python | 3.12 (시스템, venv 미사용) |
| CPU | 최소 4코어 권장 (`build_methodB_cloud.py`의 GICP가 `num_threads=4`로 고정 호출됨) |
| GPU | **선택 사항.** 아래 8절 참고 — 현재 코드는 CPU만 쓰고, GPU를 실제로 활용하려면 코드 변경이 필요할 가능성이 높다(확인 필요, 8절 참고). |
| RAM | 이 조건(스캔 21,771개, 누적 점 약 2.67억 개)은 이 저장소 안에서 가장 무거운 실험이다. 원래 방식(전체를 메모리에 들고 있다가 한 번에 concatenate)은 **5.8GB RAM에서 실제로 OOM-kill됐다**(`build_methodB_cloud.py` 상단 주석 참고). 지금 스크립트는 그 문제를 스트리밍 방식으로 고쳤지만, 그래도 **8GB 이상, 가능하면 16GB 이상**을 권장한다. |
| 디스크 | 이 비행의 bag은 수십 GB급이 될 수 있고(참고: 비슷한 조건의 다른 bag이 23GB였다), baseline/방법B 포인트클라우드(.npy)도 각각 GB 단위다. **여유 공간 60GB 이상**을 권장한다. |

---

## 3. 설치

```bash
# 1) 워크스페이스 clone
git clone <이 저장소 URL> AG-CoNav-gicp-gt-pose
cd AG-CoNav-gicp-gt-pose
git checkout gicp-gt-pose

# 2) ROS2 Jazzy + Gazebo Harmonic 등 기본 의존성 (이미 설치돼 있지 않다면)
#    PROJECT_OVERVIEW.md 10절 및 simulation_guide_jongheon.md를 따른다.
./scripts/setup_simulation.sh

# 3) small_gicp (pip, Method B 스크립트가 직접 import한다 — CPU 버전)
#    Ubuntu 24.04는 PEP 668(externally-managed-environment)로 시스템 pip에
#    직접 설치하는 걸 막는다 -- venv를 안 쓰는 이 프로젝트 관례상
#    --break-system-packages가 필요하다(실제로 --user만으로는 실패함을
#    확인함).
pip3 install --user --break-system-packages small_gicp

# 4) 워크스페이스 빌드
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`small_gicp`는 **GPU 없는 환경에서도 정상 동작한다**(순수 CPU 라이브러리,
아래 8절 참고) — 3번 단계는 GPU 유무와 무관하게 항상 필요하다.

### GPU 버전 설치 (선택 사항 — 8절의 "확인 필요" 항목들을 먼저 읽어라)

이 실험 스크립트(`run_results/build_methodB_cloud.py`)는 GLIM 전체 ROS
노드를 실행하는 게 아니라, 그 안의 정합 라이브러리(`small_gicp`)만
Python에서 직접 호출한다. 이게 왜 중요한지, GPU 패키지를 깔면 실제로
빨라지는지는 **8절에서 조사한 내용을 반드시 먼저 읽어라** — 결론만
요약하면 **지금 코드 그대로는 CUDA 패키지를 깔아도 가속되지 않을
가능성이 높다.**

그래도 조사해서 확인한 사실은 다음과 같다(설치 자체는 GPU 없는 이
브랜치 작성 환경에서 검증하지 못했다):

```bash
# koide3 PPA (glim/gtsam_points/small_gicp 배포처) — 이미 이 소스가
# 등록돼 있다면 생략. 정확한 최초 키 등록 명령은 이 문서 작성 환경에서
# 확인하지 못했다(확인 필요) — 아래는 결과로 남아있는 소스 파일 내용이다.
cat /etc/apt/sources.list.d/koide3_ppa.list
# deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2404 ./
# 없다면 GLIM 공식 저장소(https://github.com/koide3/glim)의 설치 안내를 따라
# PPA와 서명 키를 등록해라.

sudo apt update
# CUDA 12.6 또는 13.1 중 설치된 CUDA 툴킷 버전에 맞는 쪽 하나만:
sudo apt install libgtsam-points-cuda12.6-dev   # 또는 libgtsam-points-cuda13.1-dev
```

**확인 필요 (GPU 머신에서 실제 검증 못 함)**:
- `libgtsam-points-cuda*-dev`는 **C++ 라이브러리만 설치하고 Python
  바인딩이 없다**(`dpkg -L libgtsam-points-dev`로 확인, CUDA 버전도 동일한
  방식으로 빌드될 것으로 추정). 즉 `import gtsam_points`가 되는 pip
  패키지는 존재하지 않는다(`pip3 index versions gtsam_points` → 없음).
- `small_gicp`(pip, 이 실험이 실제로 쓰는 것) 자체는 PyPI에 CUDA 버전이
  없고, 설치된 버전의 Python API(`help(small_gicp.align)`)에도 GPU/CUDA
  파라미터가 없다 — `num_threads`(CPU 스레드 수)만 있다.
- **따라서 위 apt 패키지를 설치하는 것만으로는 `build_methodB_cloud.py`가
  GPU를 쓰게 되지 않을 가능성이 높다.** 실제로 GPU 가속을 받으려면 (a)
  `gtsam_points`의 CUDA 팩터를 C++에서 직접 호출하는 코드를 새로 작성하거나,
  (b) 이 스크립트 대신 GLIM 전체 ROS 노드(`ros-jazzy-glim-ros-cuda12.6`
  또는 `-cuda13.1`, apt에 존재함을 확인함)를 띄워서 GLIM 자체 파이프라인
  결과를 쓰는 방식으로 바꿔야 할 것으로 보이는데, 후자는 "방법B"의 정의
  (우리 스크립트가 GT pose를 초기값으로 직접 제어하는 것) 자체를 바꾸는
  것이라 이 실험과 같은 게 아니게 된다. GPU가 있는 환경에서 실측해서
  이 부분을 검증/수정하는 게 이 브랜치의 다음 과제로 남아있다.
- GPU를 실제로 썼는지 확인하는 로그 문구도 위와 같은 이유로 **현재 코드
  경로에서는 존재하지 않는다**(CPU에서 실행하든 GPU 패키지가 깔려 있든
  로그가 동일할 것으로 예상됨) — 확인 필요.

---

## 4. 시뮬레이션 실행 (비행 + bag 녹화)

이 저장소에 bag 자체는 들어있지 않다(용량 문제, `.gitignore`의
`bags/` 규칙). **여기서부터 새로 비행해서 bag을 만들어야 한다.**

```bash
cd AG-CoNav-gicp-gt-pose
source /opt/ros/jazzy/setup.bash
source install/setup.bash

bash run_results/run_velocity_4m_5mps_monitored.sh my_run
```

- 내부적으로 `ros2 launch agconav_test_worlds experiment.launch.py
  flight:=velocity path_file:=path_100x100_5m_4m_5mps.yaml
  cruise_speed_mps:=5.0 module_a:=false module_f:=false
  bag_output:=<repo>/bags/velocity_4m_5mps_my_run`를 실행한다(module_a/f를
  끄는 이유: 방법B는 bag의 `/drone/points`+`/tf`만 있으면 오프라인으로
  처리 가능해서, 그 두 모듈까지 같이 띄워 자원을 나눠 쓸 필요가 없다).
- 순항 기준 비행 시간은 약 570초지만, 실제로는 가속/코너 감속 등으로 더
  걸린다. 이 스크립트는 진행률 정지(5분 연속)·`/tf` 무응답(3분 연속)·
  디스크 80% 초과를 자동 감지해 안전하게 종료하고, `run_results/logs/
  velocity_4m_5mps_my_run_result.txt`에 `COMPLETED`/`STALLED`/`TF_FROZEN`/
  `TIMEOUT`/`DISK_THRESHOLD_STOP` 중 하나를 남긴다. **`COMPLETED`가 아니면
  다음 단계로 넘어가지 마라.**
- 완료되면 `bags/velocity_4m_5mps_my_run/`에 mcap bag이 생긴다.
- **알려진 이슈(실측으로 발견, 자동 복구됨)**: 이 조건의 bag은 20GB+로 커서,
  디스크가 느린 환경에서는 정상 완주(`COMPLETED`)했는데도 `ros2 bag record`가
  종료 시퀀스 중 `metadata.yaml`을 다 쓰기 전에 끊길 수 있다. 스크립트는 종료
  직후 `metadata.yaml` 존재를 확인해서 없으면 `ros2 bag reindex -s mcap`으로
  자동 복구를 시도하고 `STATUS_LOG`에 남긴다. 혹시 다른 이유로 자동 복구가
  실패했다면(로그에 "reindex 실패" 표시) 직접
  `ros2 bag reindex -s mcap bags/velocity_4m_5mps_my_run`을 실행해봐라.

---

## 5. 방법B 스크립트 실행 (정합 + 지도 생성)

```bash
python3 run_results/build_methodB_cloud.py \
  bags/velocity_4m_5mps_my_run \
  /tmp/baseline_my_run.npy \
  /tmp/methodb_my_run.npy
```

- bag의 `/tf`(map→drone/base_link, 물리엔진이 계산한 실제 GT pose)와
  `/tf_static`(base_link→os1_lidar)을 오프라인 `tf2.Buffer`로 조회해서,
  `/drone/points`의 매 스캔을 map 프레임으로 변환한다.
- `/tmp/baseline_my_run.npy` — GT pose만으로 쌓은 점군.
- `/tmp/methodb_my_run.npy` — GT pose를 초기값으로 GICP 정합해서 쌓은 점군.
- 스캔 수만 개, 점 수억 개 규모라 **오래 걸린다**(이 조건에서 실측
  기준 상당한 시간 소요 — CPU 코어 수·클럭에 따라 다르다). 진행 중
  50스캔마다 stderr에 진행 상황(GICP 성공/실패 카운트)을 출력한다.
- 메모리 걱정 없이 진행되면(RAM을 계속 다 채우지 않으면) 정상이다 — 2절의
  OOM 이슈는 이미 스트리밍 방식으로 해결되어 있다.

---

## 6. 채점 스크립트 실행 (wheel/leg FN%)

baseline과 방법B 각각 한 번씩, **총 두 번** 실행한다(둘이 서로 다른
`ROS_DOMAIN_ID`를 안 쓰면 노드가 겹치니, 반드시 하나씩 순서대로 끝내고
다음을 실행해라 — 스크립트가 시작할 때 관련 프로세스를 자동으로
정리하긴 하지만, 안전하게 순차 실행을 권장한다):

```bash
# baseline
bash run_results/run_traversability_fn_v2.sh \
  /tmp/baseline_my_run.npy baseline_my_run /tmp/baseline_my_run_fn.json

# 방법B
bash run_results/run_traversability_fn_v2.sh \
  /tmp/methodb_my_run.npy methodb_my_run /tmp/methodb_my_run_fn.json
```

- 내부적으로 Module A(`drone_elevation_mapper`)·Module F(`terrain_feature_
  calculator` + wheel/leg `traversability_verdictor`)·`elevation_map_saver`를
  띄우고, `feed_cloud.py`로 .npy 점군을 `/drone/points`에 흘려보내서 진짜
  파이프라인으로 지도를 만든다.
- `capture_nav_fn.py`가 `/wheel/nav_map`·`/leg/nav_map`을
  `run_results/gt_traversable.py`가 만드는 GT 통과가능 마스크와 대조해
  wheel/leg FN%를 계산하고 결과 JSON에 저장한다.
- 이 조건은 점 수가 아주 많아 `feed_cloud.py`가 다 보내는 데만 10분
  넘게 걸릴 수 있다 — 그래서 v2 스크립트는 캡처 타임아웃을 기본
  1500초(25분)로 넉넉하게 잡는다. 필요하면 5번째 인자로 늘려라:
  `run_traversability_fn_v2.sh <cloud> <tag> <out.json> <domain> <timeout_s>`.

---

## 7. 결과 확인

```bash
python3 -c "
import json
b = json.load(open('/tmp/baseline_my_run_fn.json'))
m = json.load(open('/tmp/methodb_my_run_fn.json'))
print('baseline wheel FN%%=%.2f leg FN%%=%.2f' % (b['wheel_FN']['FN_percent'], b['leg_FN']['FN_percent']))
print('방법B    wheel FN%%=%.2f leg FN%%=%.2f' % (m['wheel_FN']['FN_percent'], m['leg_FN']['FN_percent']))
"
```

**참고 결과값과 비교**: baseline wheel/leg FN% ≈ **13.44% / 12.60%**,
방법B wheel/leg FN% ≈ **37.26% / 30.03%** (1절 참고). 정확히 같은 수치가
나오진 않을 것이다(시뮬레이션은 결정적이지 않을 수 있고, bag마다 약간의
타이밍 차이가 생긴다) — 확인할 것은 **방향**(방법B가 baseline보다
뚜렷하게 나쁨)과 **대략적인 크기**(두 배 이상 악화)가 재현되는지다.

**GPU를 실제로 썼는지 확인하는 방법**: 8절에서 설명했듯, 현재 코드
경로(`small_gicp.align()`)는 GPU 파라미터가 아예 없어서 "GPU 사용 확인
로그"라는 게 존재하지 않는다. `nvidia-smi -l 1`을 `build_methodB_cloud.py`
실행 중에 같이 띄워서 GPU 사용률(`Volatile GPU-Util`)이 0%에 머무는지
보는 것이 지금 시점에 할 수 있는 유일한 간접 확인이다 — 0%면 GPU
패키지를 깔았어도 실제로는 안 쓰이고 있다는 뜻이다.

---

## 8. GPU 가속 관련 조사 결과 요약

3절의 "GPU 버전 설치"와 내용이 같다 — 요약하면:

- **확실한 것**: `build_methodB_cloud.py`가 쓰는 `small_gicp`(pip)
  Python API에는 GPU/CUDA 파라미터가 없다. `gtsam_points`(apt,
  `libgtsam-points-dev`/`-cuda12.6-dev`/`-cuda13.1-dev`)는 Python 바인딩이
  없는 C++ 전용 라이브러리다. 따라서 CUDA apt 패키지를 까는 것만으로는
  이 스크립트가 빨라지지 않는다.
- **확인 필요**: GPU가 있는 환경에서 (a) 위 결론이 실측으로도 맞는지
  (속도 변화 없음을 직접 확인), (b) `gtsam_points`의 CUDA 팩터를 실제로
  호출하려면 어떤 C++ API를 어떻게 바인딩해야 하는지, (c) 그 대신 GLIM
  전체 노드를 쓰는 쪽으로 아키텍처를 바꾸는 게 더 현실적인 대안인지 —
  이 세 가지는 GPU 없는 환경(이 문서를 처음 쓴 VM)에서는 검증할 수 없어
  미확정으로 남겨둔다.

자세한 조사 과정과 근거는 [`run_results/PROGRESS.md`](run_results/PROGRESS.md)
0절에 그대로 남아있다.
