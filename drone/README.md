# drone — 드론 탐색·인지

**담당:** 홍연주

상공에서 시가지를 훑어 고도맵과 **2종 통과가능성(traversability) 지도**를 만들어 지상 로봇에 배포한다.

## 범위

- kinematic 드론 스캔 경로(커버리지 패턴) 생성
- 하향 3D LiDAR → `PointCloud2` 수집 파이프라인
- 2.5D 고도맵 생성: 계단/턱/경사 자동 판정
- 지형 분류: depth/RGB → VLM 또는 분류기 (평지/rough/경사/장애물)
- **traversability 맵 2종** 산출 (4족 기준 / 4륜 기준 — 통과 조건 상이)
- 글로벌 좌표계 정렬 후 지상 로봇에 맵 배포

## 산출 인터페이스(예정)

- `/map/elevation` — 2.5D 고도맵
- `/map/trav_wheeled` — 4륜 통과 지도
- `/map/trav_legged` — 4족 통과 지도

## 구조(예정)

```
drone/
├── coverage/       # 스캔 경로 생성
├── mapping/        # 포인트클라우드 → 고도맵
├── classify/       # 지형 분류(VLM/분류기)
└── launch/
```
