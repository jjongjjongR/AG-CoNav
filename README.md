# AG-CoNav

**Terrain-Aware Aerial-Ground Cooperative Navigation for Heterogeneous Robots**

미지의 전시(戰時) 시가지에서 **드론·4륜 로봇·4족 로봇**이 팀을 이뤄 보급품을 운반하는 협동 내비게이션 시뮬레이션. 드론이 먼저 지형을 훑어 지도화하고, 지형 특성에 맞춰 4륜/4족용 이동 가능 지도를 각각 만든 뒤, 목표점마다 더 빠르고 안전한 로봇을 배정해 **최소 시간·최대 안전**으로 임무를 완수한다. 실패가 발생하면 경로와 임무를 재배분한다.

> 상태: 🚧 설계·구현 초기 (2026-07-13 킥오프 발표 준비 중)

---

## 핵심 아이디어

이종(異種) 로봇은 각자 강점 지형이 다르다. 드론은 상공에서 전체 지형을 빠르게 파악하고, 4륜(skid-steer)은 뚫린 평지에서 빠르며, 4족은 잔해·계단 같은 험지에 강하다. AG-CoNav는 이 이질성을 **하나의 통과가능성(traversability) 기반 임무 배분 문제**로 통합한다.

## 미션 플로우

| 단계 | 주체 | 내용 |
| --- | --- | --- |
| 1. 훑어보기 | 드론 | 시가지 상공을 비행하며 LiDAR 스캔 → 고도맵(2.5D) 생성. 평지/잔해/계단/경사 구분 |
| 2. 2종 지도 | 드론 | 고도맵에서 **4륜 통과 지도**와 **4족 통과 지도**를 각각 산출 (통과 조건이 다름) |
| 3. 목표 탐지 | 드론 + 지상 | 드론이 큰 그림, 4륜·4족이 지상에서 국소 탐지 → 건물 뒤·잔해 밑 등 사각지대 목표 발굴 |
| 4. 배정 | 오케스트레이터 | 목표별로 4륜 vs 4족 **도달성·비용 비교** → 더 나은 로봇 파견 |
| 5. 실행 & 재배정 | 각 로봇 | Nav2로 이동·임무 수행. 막힘/통과 실패 시 재계산 후 다른 로봇으로 재배분 |

배정 비용 = **이동시간 + 위험(경사·잔해·전복) + 통과 가능성**

---

## 기술 스택

| 계층 | 선택 | 비고 |
| --- | --- | --- |
| OS | Ubuntu 24.04 LTS | ROS2 Jazzy Tier-1 공식 지원 |
| 미들웨어 | ROS2 Jazzy Jalisco | LTS(2029.05까지), C++/Python(rclpy) 혼용 |
| 메인 시뮬 | Gazebo Harmonic | Jazzy 공식 페어링, `ros_gz_bridge` 직결 |
| RL 시뮬 | MuJoCo 3.10.x | Go2가 Menagerie 공식 포함, MJX GPU 병렬 학습 |
| 내비 | Nav2 | costmap + traversability 커스텀 레이어 |
| 매핑 | SLAM Toolbox (선택) | 정적 맵으로 대체 가능 |
| 브릿지/시각화 | ros_gz_bridge, RViz2 | |
| LLM | OpenAI API | 오케스트레이션 재배분 비교용 |

### 로봇 플랫폼

- **드론** — Gazebo 기본 멀티콥터(kinematic) + 하향 3D GPU LiDAR. 확장 시 PX4 x500
- **4륜** — Clearpath Husky A300 (skid-steer). Clearpath 공식 시뮬 Jazzy+Harmonic
- **4족** — Unitree Go2 + CHAMP 보행(학습 없이 즉시). 고도화로 MuJoCo RL(PPO)

---

## 저장소 구조

```
AG-CoNav/
├── sim_env/          # Gazebo 환경·시가지 맵·스토리·로봇 스폰 (채현우)
├── drone/            # 드론 커버리지 스캔·고도맵·지형분류·2종 traversability (홍연주)
├── ugv_husky/        # 4륜 자율: 지상 탐지 + Nav2 (이수빈)
├── quad_go2/         # 4족 자율: CHAMP + 지상 탐지 + Nav2 (채현우·이수빈)
├── quad_rl/          # 4족 RL 고도화: MuJoCo + PPO, sim-to-sim (이종헌)
├── orchestration/    # 탐지융합·로봇선택·임무배분·재계획·FSM·LLM (이종헌)
├── integration/      # 멀티로봇 ROS2 그래프·메시지 정의·통합 런치·로깅
├── experiments/      # 지표·Ablation·결과·데모·논문/발표 자료
└── docs/             # 아키텍처·설계 노트·회의록
```

각 폴더에 세부 README가 있다.

---

## 역할 분담 (R&R)

| 모듈 | 담당 |
| --- | --- |
| 시뮬 환경/스토리 | **채현우** |
| 드론 탐색·인지 | **홍연주** |
| 4륜 자율 (탐지 + Nav2) | **이수빈** |
| 4족 자율 (CHAMP + 탐지 + Nav2) | **채현우 / 이수빈** |
| 4족 RL 고도화 (MuJoCo + PPO) | **이종헌** |
| 오케스트레이션 (탐지융합·로봇선택·LLM) | **이종헌** |
| 시스템 통합 | 채현우·이수빈 (자기 모듈 연장) |
| 실험/평가/문서/발표 | 전원 분담 |

---

## 프로젝트 우선순위

1. 구현 가능성  2. 시뮬레이션 안정성  3. 연구 질문과의 연결성  4. 발표·논문화 가능성  5. 확장성

## 평가 지표 (예정)

임무 완료시간, 주행 실패율, 위험구간 선택률, 로봇 선택 정확도, 배분 효율.
**Ablation**: 지형정보 사용/미사용 · 최적 배분/규칙 배분 · LLM 재배분/규칙 재배분.

---

## 시작하기

> 상세 셋업은 각 모듈 README 참조. 아래는 뼈대(추후 확정).

```bash
# 요구사항: Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic
git clone <repo-url> AG-CoNav
cd AG-CoNav

# 워크스페이스 빌드 (ROS2 패키지 배치 후)
colcon build --symlink-install
source install/setup.bash

# 통합 런치 (TBD)
# ros2 launch integration ag_conav.launch.py
```

## 기여 방법

브랜치 전략과 PR 규칙은 [CONTRIBUTING.md](CONTRIBUTING.md) 참조. 요약: `main` 보호, 모듈별 브랜치 + PR 리뷰.

## 라이선스

[MIT](LICENSE)
