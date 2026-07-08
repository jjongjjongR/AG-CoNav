# 기여 가이드

## 저장소 전략

- **단일 레포, 폴더 분리.** 모듈별 폴더(`sim_env`, `drone`, `ugv_husky`, `quad_go2`, `quad_rl`, `orchestration`, `integration`, `experiments`, `docs`)로 나눈다.

## 브랜치 전략

- `main` — 보호 브랜치. 직접 push 금지, PR로만 병합.
- 모듈별 작업 브랜치에서 개발 후 PR.

브랜치 이름 규칙:

```
<모듈>/<작업>        예) drone/elevation-map, orchestration/assign-matrix
fix/<이슈>          예) fix/nav2-costmap-crash
```

## PR 규칙

- 모든 변경은 PR을 통해 병합하고 **최소 1인 리뷰**를 받는다.
- PR 제목은 `[모듈] 요약` 형식 (예: `[drone] 2.5D 고도맵 생성`).
- 본문에 변경 목적·테스트 방법·관련 이슈를 적는다.
- 자기 모듈 외 폴더를 건드리면 해당 담당자를 리뷰어로 지정.

## 커밋 메시지 (권장)

```
<모듈>: <요약>

- 상세 1
- 상세 2
```

## 담당 모듈

| 폴더 | 담당 |
| --- | --- |
| sim_env | 채현우 |
| drone | 홍연주 |
| ugv_husky | 이수빈 |
| quad_go2 | 채현우 · 이수빈 |
| quad_rl | 이종헌 |
| orchestration | 이종헌 |
| integration | 채현우 · 이수빈 |
| experiments / docs | 전원 |

## 코드 스타일

- ROS2 노드는 C++/Python(rclpy) 혼용. 각자 모듈 언어 컨벤션을 따르고, 공용 메시지는 `integration/msgs`에서 정의.
