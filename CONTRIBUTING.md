# 기여 가이드 (CONTRIBUTING)

> AG-CoNav 팀 기여 규칙. 프로젝트 개요·아키텍처·공통 규약(단위·프레임·해상도·TF 소유권·시간·QoS)은 **[README.md](README.md)** 를 기준으로 한다.

---

## 1. 브랜치 전략

- 각자 **자기 이름 브랜치**에서 작업한다. (예: `jongheon`, `brian`, …)
- 작업 단위로 `main`에 **PR**을 올린다.
- `main`은 항상 빌드 가능한 상태를 유지한다.

## 2. PR 정책

- **최소 1명 리뷰 승인** 후 머지.
- **셀프 머지 허용**(리뷰가 지연될 때).
- **인터페이스 변경**(토픽·메시지·TF·공통 규약) PR은 README도 함께 수정하고 전원에게 공지한다.

## 3. 커밋 메시지 규칙

형식: **`[type] 설명`**  (예: `[docs] README.md 수정`)

| 타입 | 설명 | SemVer |
| --- | --- | --- |
| **feat** | 새로운 기능 추가 | MINOR |
| **fix** | 버그 수정 | PATCH |
| **docs** | 문서 수정 (README, 주석 등) | 없음 |
| **style** | 포맷팅·세미콜론 등 동작 변화 없는 변경 | 없음 |
| **refactor** | 기능·버그 변화 없는 리팩토링 | 없음 |
| **test** | 테스트 코드 추가·수정 | 없음 |
| **chore** | 빌드·패키지 매니저 등 관리 작업 | 없음 |
| **perf** | 성능 개선 | PATCH 또는 없음 |

## 4. 패키지 · 모듈 소유권

| 모듈 | 담당 | 패키지 |
| --- | --- | --- |
| A 드론 지도 생성 | 홍연주 | `agconav_drone` |
| F 지형 주행성 분석 | 이종헌 | `agconav_traversability` |
| B 지상 위치추정 | 이수빈 | `agconav_localization` |
| C 지상 Nav2 이동 | 이수빈 | `agconav_navigation` |
| D 지상 지도 누적 | 채현우 | `agconav_ground_mapping` |
| E 모든 지도 병합 | 채현우 | `agconav_map_fusion` |
| 공통 인프라 | 이종헌 | `agconav_worlds`·`agconav_description`·`agconav_gz_bridge`·`agconav_bringup` |

- 모듈끼리는 **토픽으로만 결합**한다. 다른 패키지의 내부 코드를 직접 import/호출하지 않는다.
- 남의 패키지를 고쳐야 하면 담당자와 PR로 협의한다. `agconav_bringup`만 전체를 안다.

## 5. 코드 규약

- **설명 가능한 것만 넣는다.** 좌표 변환·용어·라이브러리·툴 전부 스스로 설명 가능해야 한다. (AI 추천만 보고 넣지 않기. 연동·통합 방법은 도움받아도 됨.)
- 공통 규약(단위·프레임·해상도·`elevation`/NaN·TF 소유권·시간·QoS)은 **README §3** 준수. 함부로 바꾸지 않는다.
- 코드에는 **상대 토픽 이름**(`points`, `odom`, …)을 쓰고 네임스페이스로 해석되게 한다. `/wheel/points`를 직접 적지 않는다.
- 전 노드 `use_sim_time: true`.
- Python `ament_flake8`, C++ `ament_cpplint` 통과 권장.

## 6. 라이선스

- **미지정**(내부 프로젝트). 각 `package.xml`의 `<license>` 필드는 `TODO`로 둔다.
