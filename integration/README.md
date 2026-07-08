# integration — 시스템 통합

**담당:** 채현우 · 이수빈 (자기 모듈 연장)

모듈을 하나의 멀티로봇 ROS2 그래프로 묶는다.

## 범위

- 멀티로봇 ROS2 그래프: 네임스페이스, tf 트리
- 모듈 간 메시지 정의: 맵 · 탐지 · 할당 · 상태 토픽
- 통합 런치, 시간 동기화, 로깅(rosbag)

## 메시지·토픽 규약(초안)

| 토픽 | 타입 | 발행 | 구독 |
| --- | --- | --- | --- |
| `/map/elevation` | 고도맵 | drone | orchestration |
| `/map/trav_wheeled` | 4륜 통과맵 | drone | ugv_husky |
| `/map/trav_legged` | 4족 통과맵 | drone | quad_go2 |
| `/detections/*` | 목표·아군 탐지 | 각 로봇 | orchestration |
| `/mission/assignment` | 로봇별 배정 | orchestration | 각 로봇 |
| `/events/failure` | 통과 실패 | 각 로봇 | orchestration |

> 정확한 메시지 타입/필드는 확정 후 이 표를 갱신.

## 구조(예정)

```
integration/
├── msgs/           # 커스텀 메시지 정의
├── launch/         # 통합 런치
└── config/         # tf·네임스페이스·QoS
```
