# quad_rl — 4족 RL 고도화 (MuJoCo + PPO)

**담당:** 이종헌

MuJoCo에서 Go2를 PPO로 학습해 CHAMP 보행을 대체/병행한다. 학습된 정책을 ONNX로 내보내 ROS2 추론 노드로 Gazebo에 이식한다(sim-to-sim).

## 범위

- MuJoCo Menagerie Go2 + PPO 학습
- 보상: 속도추종 · 자세 · 에너지 · 낙상 + **지형 커리큘럼** + 도메인 랜덤화
- sim-to-sim 이식: ONNX → ROS2 추론 노드 → Gazebo (CHAMP 대체/병행)

## 파이프라인(예정)

```
학습(MuJoCo/MJX, PPO) → 정책 평가 → ONNX export
   → ROS2 추론 노드 → Gazebo Go2 관절 명령 → 검증
```

## 구조(예정)

```
quad_rl/
├── envs/           # MuJoCo Go2 환경·커리큘럼
├── train/          # PPO 학습 스크립트·설정
├── export/         # ONNX 변환
└── ros2_infer/     # ROS2 추론 노드
```

## 요구사항

- MuJoCo 3.10.x, (MJX GPU 병렬 학습 권장)
