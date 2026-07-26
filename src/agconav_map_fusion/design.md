# E. 모든 지도 병합 — 설계 문서

이 문서는 드론(모듈 A)의 2.5D 지도와 지상 로봇(모듈 D)의 wheel·leg 2.5D 지도를 하나의 통합 2.5D 지도로 병합하는 ROS2 패키지의 설계 문서다. 이 문서에 정의된 인터페이스, 파라미터, QoS 조건, 병합 규칙을 정확히 따라서 구현해야 한다.

---

## 1. 책임

모듈 A의 드론 2.5D 지도와 모듈 D의 wheel·leg 2.5D 지도를 공통 `map` 좌표와 `0.10 m/cell` 격자 규약에 맞춰 검증·배치하고, 합집합 범위·`wheel > leg > 드론` 우선순위·`NaN` 보존 규칙을 적용하여 **세 지도의 생성 완료 상태가 모두 수신된 시점에 1회** 통합 2.5D 지도로 병합하며, 완성된 지도를 제공하고 저장한다.

1. 드론 2.5D 지도 입력
2. wheel 2.5D 지도 입력
3. leg 2.5D 지도 입력
4. 드론·wheel·leg 세 지도의 생성 완료 상태를 모두 확인할 때까지 대기
5. 세 지도의 frame, 원점, 해상도, 범위 확인
6. 세 지도의 동일 해상도(`0.10 m/cell`) 검증
7. 세 입력 지도의 합집합 범위로 공통 출력 격자 생성
8. 중복 셀에서 `wheel > leg > 드론` 순서로 우선 적용
9. 유효값을 `NaN`으로 덮지 않고, 모두 미관측인 셀만 `NaN` 유지
10. 통합 2.5D 지도 생성
11. 완성된 통합 지도를 1회 제공, 저장 (이후 입력 지도가 갱신되어도 재병합하지 않음)

### 병합 실행 방식 — 핵심 규칙

- **병합은 반복 갱신되지 않는다.** 드론·wheel·leg 세 완료 상태가 모두 수신된 시점에 **딱 1회** 병합·저장하고 끝난다.
- 세 완료 상태 수신 이후 입력 지도가 추가로 갱신되어도 **재병합하지 않는다.**
- **노드 실행 조건**(launch가 실행되어 노드가 켜지는 시점)과 **병합 트리거 조건**(세 완료 상태가 모두 수신되어 실제 병합 로직이 실행되는 시점)은 서로 다른 단계이므로 구분한다. 노드는 launch 시점에 켜지지만, 병합 로직 자체는 트리거 조건이 충족될 때까지 대기한다.

> 참고: wheel/leg의 "이동 완료"는 모듈 C가 발행하지만, 모듈 E는 C를 직접 구독하지 않는다. 모듈 D가 C의 완료 신호를 받아 자신의 `elevation_map_status`로 다시 발행하므로, E는 D의 상태만 구독한다 (CONTRIBUTING 4: 모듈끼리는 토픽으로만 결합, 남의 패키지 직접 참조 금지).

---

## 2. 범위 밖

| 범위 밖 항목 | 이유 |
| --- | --- |
| Gazebo 월드 생성 | `agconav_worlds`의 월드 구성에서 처리 |
| 로봇 모델 스폰 | 로봇 bringup launch에서 처리 |
| Gazebo 데이터를 ROS 2로 브리지 | `agconav_gz_bridge`에서 처리 |
| `/clock` 생성 | Gazebo가 유일한 시간 소스 |
| 드론 원시 점군 처리 | 모듈 A가 드론 2.5D 지도를 생성한 뒤 제공 |
| wheel·leg 원시 점군 처리 | 모듈 D가 로봇별 2.5D 지도를 생성한 뒤 제공 |
| 드론 지도 생성 | 모듈 A 책임 |
| 지상 로봇 위치 추정 | 모듈 B 책임 |
| 지상 로봇 Nav2 이동 | 모듈 C 책임 |
| wheel·leg 지도 생성 | 모듈 D 책임 |
| 로봇 위치 추정 오차 보정 | 모듈 B가 각 지도를 공통 `map` 좌표에 정렬할 수 있는 위치와 TF를 제공 |
| 새로운 SLAM 알고리즘 개발 | 기존 지도 결과를 병합하는 통합 엔지니어링이 목적 |
| wheel·leg 주행 가능 맵 생성 | 모듈 F가 드론 2.5D 지도에서 생성 |
| 목표 탐지·임무 배정 | 현재 핵심 범위에서 제외 |
| 지도 병합 후 로봇 이동 | 모듈 C 책임 |
| 원시 센서 데이터 보정 | 각 지도 생성 모듈이 처리한 결과를 입력으로 사용 |

---

## 3. 필요한 데이터 / 입력

- 드론 2.5D 높이 지도
- wheel 2.5D 높이 지도
- leg 2.5D 높이 지도
- 드론 지도 생성 완료 상태
- wheel 지도 생성 완료 상태
- leg 지도 생성 완료 상태
- 시뮬레이션 시간
- 지도 병합 설정 (우선순위 규칙, 해상도 기준값 등)
- 노드 실행 조건 (launch 실행 시점)
- 병합 트리거 조건 (세 완료 상태 모두 수신 시점)

### 입력 제공자

- 드론 2.5D 높이 지도 = 모듈 A (드론 지도 생성)
- wheel 2.5D 높이 지도 = 모듈 D (지상 로봇 주변 지형 지도 누적)
- leg 2.5D 높이 지도 = 모듈 D (지상 로봇 주변 지형 지도 누적)
- 드론 지도 생성 완료 상태 = 모듈 A
- wheel 지도 생성 완료 상태 = 모듈 D
- leg 지도 생성 완료 상태 = 모듈 D
- 시뮬레이션 시간 = Gazebo `/clock`
- 노드 실행 조건 = 사용자가 모듈 E launch 실행
- 병합 트리거 조건 = 드론·wheel·leg 세 완료 상태가 모두 수신됨

### 연결

드론 지도 + wheel 지도 + leg 지도 → (세 완료 상태 모두 수신 대기) → 공통 격자 규약 검증 → 합집합 출력 격자 생성 → 중복 셀 우선순위(`wheel > leg > 드론`) 적용 → 통합 2.5D 지도 생성 → 저장·제공(1회)

---

## 4. 변환 찾기

1. 세 지도(frame, 해상도, 원점, 범위) → 검증 결과 : 값을 비교해 병합 가능 여부 판단
2. 검증 통과한 세 지도의 범위 → 출력 격자의 원점·크기 : 합집합 범위 계산
3. 출력 격자 + 세 지도의 셀별 값 → 중복 셀 우선순위 적용 : `wheel > leg > 드론` 순, 유효값은 `NaN`으로 덮지 않음
4. 병합된 셀 값들 → 통합 2.5D 지도 : `elevation` 레이어로 조립
5. 통합 2.5D 지도 → 지도 파일 : rosbag2 `mcap` 형식으로 저장 (병합 완료 시 1회)

---

## 5. 출력

- 통합 2.5D 높이 지도 : 드론·wheel·leg 지도를 하나의 공통 `map` 좌표에 병합한 최종 결과
- 저장된 통합 지도 : 실행 종료 후 다시 불러올 수 있는 최종 지도 결과물
- 지도 병합 완료 상태 : 세 지도의 최종 병합이 완료됐는지 판단하기 위한 상태
- 지도 병합 오류 상태 : 입력 지도 누락, frame 불일치 또는 해상도 처리 실패를 판단하기 위한 상태

### 출력 스펙

- 좌표 기준: `map`
- 셀 위치 단위: m
- 높이 단위: m
- 기본 해상도: `0.10 m/cell`
- 핵심 레이어: `elevation`
- 출력 범위: 세 입력 지도의 유효 범위를 모두 포함하는 합집합
- 중복 셀 처리: `wheel > leg > 드론` 우선순위
- 미관측 셀 처리: 유효값을 `NaN`으로 덮지 않으며, 세 지도 모두 미관측인 셀만 `NaN` (미관측 셀과 실제 높이 0m는 반드시 구분)
- 해상도 처리: 세 입력 모두 `0.10 m/cell`이어야 하며 불일치 시 병합하지 않고 오류 처리 (임의 리샘플링 금지)
- 출력 지도 원점: 합집합 범위를 표현할 수 있도록 공통 `map` 좌표에서 계산
- 필수 보존 정보: frame, 해상도, 지도 크기·원점, `elevation`, `NaN`
- 병합 실행 방식: 세 완료 상태가 모두 수신된 시점에 단 1회 병합·저장 (반복 갱신 없음)
- 중복 셀 처리는 입력 순서에 의존하지 않고 항상 결정적(deterministic)이어야 함
- 비정상적인 높이 값(이상치)이 통합 지도 전체를 왜곡하지 않도록 셀 단위로만 반영

### 출력 소비자

- 통합 2.5D 높이 지도 : RViz2, 검증 담당자
- 저장된 통합 지도 : 프로젝트 최종 결과 검증, 발표·시연
- 지도 병합 완료 상태 : 사용자, 검증 담당자
- 지도 병합 오류 상태 : 사용자, 검증 담당자
- 현재 A~F 모듈 중 통합 2.5D 지도를 직접 구독하는 후속 모듈은 없음 (향후 확장 여지만 있음)

---

## 6. 책임 상자 (노드 단위)

### 6-1. 입력 지도 수집 책임 — `map_merge_collector`

- **입력:** 드론/wheel/leg 2.5D 지도, 각 지도 생성 완료 상태
- **출력:** 병합 대상 지도 세트(최신본), 병합 실행 트리거
- **하는 일**
  - 세 지도 토픽 구독
  - 각 지도의 최신 상태 저장
  - 지도 생성 완료 상태 함께 확인
  - 입력 누락(지도 미수신) 감지
  - 드론·wheel·leg 세 완료 상태가 모두 True가 될 때까지 대기, 모두 완료되면 병합 실행을 **1회** 트리거

### 6-2. 입력 검증 책임 — `map_merge_validator`

- **입력:** 병합 대상 지도 세트
- **출력:** 검증 통과 지도 세트 또는 병합 오류 상태
- **하는 일**
  - frame이 모두 `map`인지 확인
  - 해상도가 모두 `0.10 m/cell`인지 확인
  - `elevation` 레이어 존재 여부 확인
  - 하나라도 조건 불충족 시 병합 중단 및 오류 상태 발행

### 6-3. 출력 격자 생성 책임 — `map_merge_grid_builder`

- **입력:** 검증 통과 지도 세트
- **출력:** 병합용 출력 격자(원점, 크기)
- **하는 일**
  - 세 지도의 범위를 비교
  - 합집합 범위 계산
  - 출력 격자의 원점과 크기 결정
  - 메모리 한계 초과 여부 확인

### 6-4. 지도 병합 책임 — `elevation_map_merger`

- **입력:** 출력 격자, 검증 통과 지도 세트
- **출력:** 통합 2.5D 지도
- **하는 일**
  - 각 지도의 셀 값을 출력 격자에 배치
  - 중복 셀에서 `wheel > leg > 드론` 순서로 우선 적용 (입력 순회 순서에 의존하지 않는 결정적 처리)
  - 유효값을 `NaN`으로 덮지 않음
  - 세 지도 모두 미관측인 셀만 `NaN` 유지
  - 통합 `elevation` 레이어 완성

### 6-5. 지도 저장 책임 — `merged_elevation_map_saver`

- **입력:** 통합 2.5D 지도, 저장 경로
- **출력:** 저장된 통합 지도 파일
- **하는 일**
  - 통합 지도를 전달받음 (병합 완료 시 1회)
  - rosbag2 `mcap` 형식으로 변환
  - 지정 경로에 저장

### 6-6. 완료·오류 상태 제공 책임 (`elevation_map_merger` 내부 기능으로 통합)

- **입력:** 병합 진행 상황, 검증 결과
- **출력:** 지도 병합 완료 상태, 지도 병합 오류 상태
- **하는 일**
  - 병합 성공 시 완료 상태 발행 (1회)
  - 검증 실패·입력 누락 시 오류 상태 발행
  - 사용자와 검증 담당자가 확인 가능하게 함

---

## 7. 상자 사이 연결

- 입력 지도 수집 책임 → 입력 검증 책임 (병합 트리거 발생 시)
- 입력 검증 책임 → 출력 격자 생성 책임
- 입력 검증 책임 → 완료·오류 상태 제공 책임 (검증 실패 시 오류 경로)
- 출력 격자 생성 책임 → 지도 병합 책임
- 지도 병합 책임 → 지도 저장 책임
- 지도 병합 책임 → 완료·오류 상태 제공 책임
- 지도 병합 책임 → RViz2, 검증 담당자

### 7-1. 구현 단위

| 책임 | 구현 방식 | 노드 이름 |
| --- | --- | --- |
| 입력 지도 수집 | ROS 노드, 커스텀 | `map_merge_collector` |
| 입력 검증 | ROS 노드, 커스텀 (map_merge_collector에 통합 가능, 팀 결정 필요) | `map_merge_validator` |
| 출력 격자 생성 | ROS 노드, 커스텀 | `map_merge_grid_builder` |
| 지도 병합 | ROS 노드, grid_map 병합 로직 커스텀 구현 | `elevation_map_merger` |
| 지도 저장 | ROS 노드, 모듈 A/D `elevation_map_saver` 구조 재사용 | `merged_elevation_map_saver` |
| 완료·오류 상태 제공 | `elevation_map_merger` 내부 기능 | - |

추가 구성 요소: 없음 (Gazebo/브릿지 의존 없음, 순수 ROS 노드 간 통신)

### 7-2. 데이터 흐름

- 드론 지도(`/drone/elevation_map`) + wheel 지도(`/wheel/elevation_map`) + leg 지도(`/leg/elevation_map`) → `map_merge_collector` → `map_merge_validator`
- `map_merge_validator`(통과) → `map_merge_grid_builder`
- `map_merge_validator`(실패) → `elevation_map_merger`(오류 상태 발행)
- `map_merge_grid_builder` → `elevation_map_merger`
- `elevation_map_merger` → `merged_elevation_map_saver` → 파일 시스템
- `elevation_map_merger` → RViz2, 검증 담당자

---

## 8. ROS 인터페이스 결정

### 8-1. 모듈 A/D → 지도 수집

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 모듈 A(`drone_elevation_mapper`), 모듈 D(`ground_elevation_mapper` ×2) |
| 수신 책임 | 입력 지도 수집(`map_merge_collector`) |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/drone/elevation_map`, `/wheel/elevation_map`, `/leg/elevation_map` (기존 이름 그대로 구독) |
| 데이터 의미 | 각 모듈이 만든 2.5D 높이 지도 |
| 타입 | `grid_map_msgs/msg/GridMap` |
| 필수 레이어 | `elevation` |

### 8-2. 모듈 A/D → 완료 상태 확인

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 모듈 A, 모듈 D |
| 수신 책임 | 입력 지도 수집(`map_merge_collector`) |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/drone/elevation_map_status`, `/wheel/elevation_map_status`, `/leg/elevation_map_status` |
| 데이터 의미 | 각 로봇/모듈의 지도 생성 완료 여부 |
| 타입 | `std_msgs/msg/Bool` (모듈 D는 이미 이 타입으로 확정됨) |
| 병합 트리거 조건 | 세 상태가 모두 `True`가 된 시점, 1회만 트리거 |

### 8-3. 지도 수집 → 지도 검증

| 항목 | 내용 |
| --- | --- |
| 처리 책임 | 지도 검증(`map_merge_validator`) |
| 연결 방식 | 내부 함수 호출 (또는 내부 Topic, 노드 분리 여부에 따라 결정 — 노드 분리 시 `/merge/collected_maps`) |
| 검증 항목 | frame `map` 일치, 해상도 `0.10 m/cell` 일치, `elevation` 레이어 존재 |
| 실패 시 처리 | 병합 중단, 오류 상태 발행 |

### 8-4. 검증 통과 지도 → 출력 격자 생성

| 항목 | 내용 |
| --- | --- |
| 처리 책임 | 출력 격자 생성(`map_merge_grid_builder`) |
| 연결 방식 | 내부 함수 호출 |
| 계산 결과 | 합집합 범위의 출력 격자 원점, 크기 |
| 실패 조건 | 출력 격자가 메모리 한계 초과 |

### 8-5. 출력 격자 + 검증 통과 지도 → 지도 병합

| 항목 | 내용 |
| --- | --- |
| 처리 책임 | 지도 병합(`elevation_map_merger`) |
| 병합 규칙 | `wheel > leg > 드론` 순 우선, 유효값은 `NaN`으로 덮지 않음, 셋 다 미관측인 셀만 `NaN` |
| 결과 | 통합 `elevation` 레이어를 가진 GridMap |

### 8-6. 지도 병합 → 저장/RViz2/검증 담당자

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 지도 병합(`elevation_map_merger`) |
| 수신 책임 | 지도 저장, RViz2, 검증 담당자 |
| 수신 구현 | `merged_elevation_map_saver`, RViz2 |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/merged/elevation_map` |
| 데이터 의미 | 드론·wheel·leg 지도를 병합한 최종 통합 2.5D 지도 |
| 타입 | `grid_map_msgs/msg/GridMap` |
| 필수 레이어 | `elevation` |
| 발행 주기 | 병합 트리거(세 완료 상태 모두 수신) 시 1회만 발행 (반복 갱신 없음) |

### 8-7. 지도 병합 → 완료·오류 상태 제공

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 지도 병합(`elevation_map_merger`) |
| 수신 책임 | 사용자, 검증 담당자 |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/merged/merge_status`, `/merged/merge_error` |
| 데이터 의미 | 병합 완료 여부 / 입력 누락·frame 불일치·해상도 불일치 등 오류 원인 |
| 타입 | 확정 필요 — 구현 시 우선 `std_msgs/msg/Bool`(merge_status)로 가정, 오류 원인은 `std_msgs/msg/String`(merge_error)로 가정 |

### 8-8. 지도 병합 → 지도 저장

| 항목 | 내용 |
| --- | --- |
| 출력 책임 | 지도 저장(`merged_elevation_map_saver`) |
| 출력 대상 | 파일 시스템 |
| 연결 방식 | `/merged/elevation_map` 토픽 구독, 병합 완료(1회 발행) 시 저장 |
| 저장 폴더 | Parameter `output_directory` |
| 파일 이름 | Parameter `map_name` |
| 저장 형식 | Parameter `output_format` (`mcap`) |

---

## 9. 좌표, 시간, 통신 조건

### 9-1. 모듈 A/D → 입력 지도 수집(`map_merge_collector`)

`/drone/elevation_map`, `/wheel/elevation_map`, `/leg/elevation_map`

| 조건 | 결정 |
| --- | --- |
| 타입 | `grid_map_msgs/msg/GridMap` |
| frame_id | `map` |
| stamp | 각 모듈이 지도를 마지막으로 갱신한 시뮬레이션 시각 |
| 필수 레이어 | `elevation` |
| 해상도 | `0.10 m/cell` (검증 대상) |
| 구독 주기 | 각 모듈 발행 주기(기본 1Hz)에 맞춤 |
| Reliability | reliable |
| Durability | transient local |
| History | keep last |
| Depth | 1 |

### 9-2. 모듈 A/D → 완료 상태 확인(`map_merge_collector`)

`/drone/elevation_map_status`, `/wheel/elevation_map_status`, `/leg/elevation_map_status`

| 조건 | 결정 |
| --- | --- |
| 타입 | `std_msgs/msg/Bool` |
| 구독 시점 | 지속 구독, 세 개 모두 True가 되면 병합 트리거 |
| Reliability | reliable |
| Durability | transient local |

### 9-3. 지도 검증(`map_merge_validator`) 조건

| 조건 | 결정 |
| --- | --- |
| frame 검증 | 세 지도 모두 `map`이어야 함 |
| 해상도 검증 | 세 지도 모두 `0.10 m/cell`이어야 함 |
| 레이어 검증 | 세 지도 모두 `elevation` 레이어 보유해야 함 |
| 실패 처리 | 병합 중단, `/merged/merge_error` 발행, 임의 리샘플링 금지 |

### 9-4. 출력 격자 생성(`map_merge_grid_builder`) 조건

| 조건 | 결정 |
| --- | --- |
| 출력 범위 | 세 입력 지도의 합집합 |
| 출력 해상도 | `0.10 m/cell` |
| 원점 계산 기준 | 공통 `map` 좌표 |
| 메모리 확인 | 출력 격자 크기가 한계를 초과하면 오류 처리 |

### 9-5. 지도 병합(`elevation_map_merger`) → 저장/RViz2/검증 담당자

`/merged/elevation_map`

| 조건 | 결정 |
| --- | --- |
| 타입 | `grid_map_msgs/msg/GridMap` |
| frame_id | `map` |
| stamp | 병합이 완료된 시뮬레이션 시각 |
| 필수 레이어 | `elevation` |
| 높이 단위 | m |
| 해상도 | `0.10 m/cell` |
| 지도 범위 | 세 입력 지도의 합집합 |
| 중복 셀 우선순위 | `wheel > leg > 드론` |
| 미관측 처리 | 세 지도 모두 `NaN`인 셀만 `NaN` |
| 발행 주기 | 병합 트리거(세 완료 상태 모두 수신) 시 **1회만** (반복 갱신 없음) |
| Reliability | reliable |
| Durability | transient local |
| History | keep last |
| Depth | 1 |

### 9-6. 지도 병합(`elevation_map_merger`) → 완료·오류 상태

`/merged/merge_status`, `/merged/merge_error`

| 조건 | 결정 |
| --- | --- |
| 타입 | `std_msgs/msg/Bool` (merge_status), `std_msgs/msg/String` (merge_error, 가정) |
| 발행 시점 | 병합 성공/실패 시 1회 |
| Reliability | reliable |
| Durability | transient local |

### 9-7. 지도 병합(`elevation_map_merger`) → 지도 저장(`merged_elevation_map_saver`)

| 조건 | 결정 |
| --- | --- |
| 입력 토픽 | `/merged/elevation_map` |
| 저장 형식 | rosbag2 `mcap` |
| 저장 시점 | 병합 완료(1회 발행) 수신 시 1회 저장 |
| use_sim_time | `true` |

지도 저장 Parameter:

| Parameter | 의미 | 기본값 |
| --- | --- | --- |
| `input_topic` | 저장할 지도 토픽 | `/merged/elevation_map` |
| `output_directory` | 저장 폴더 | 프로젝트 `maps/` |
| `map_name` | 파일 이름 | `merged_elevation_map` |
| `output_format` | 저장 형식 | `mcap` |
| `use_sim_time` | Gazebo 시간 사용 | `true` |

---

## 10. 실행 묶음 (launch 구조)

```
elevation_map_merge.launch.py
 │
 ├─ 입력 지도 수집(map_merge_collector)
 │   └─ map_merge_collector.yaml
 │
 ├─ 입력 검증(map_merge_validator)
 │   └─ map_merge_validator.yaml
 │
 ├─ 출력 격자 생성(map_merge_grid_builder)
 │   └─ map_merge_grid_builder.yaml
 │
 ├─ 지도 병합(elevation_map_merger)
 │   └─ elevation_map_merger.yaml
 │
 └─ 지도 저장(merged_elevation_map_saver)
     └─ merged_elevation_map_saver.yaml
```

---

## 11. 전제조건

| 분류 | 전제조건 |
| --- | --- |
| 입력 지도 | 드론·wheel·leg 2.5D 지도가 준비되어야 함 |
| 지도 완료 상태 | 각 지도 생성 완료 여부를 확인할 수 있어야 함 |
| 좌표 | 세 지도가 공통 전역 프레임 `map`으로 표현되어야 함 |
| 지도 원점 | 세 지도의 원점과 `map` 원점의 관계를 알 수 있어야 함 |
| 단위 | 세 지도 모두 위치와 높이에 m 단위를 사용해야 함 |
| 레이어 | 세 지도에 병합할 `elevation` 레이어가 존재해야 함 |
| 해상도 | 세 입력 지도 모두 `0.10 m/cell`이어야 하며 불일치 시 오류 처리해야 함 |
| 지도 범위 | 각 입력 지도의 크기와 범위 정보를 읽고 합집합 출력 범위를 계산할 수 있어야 함 |
| 미관측 값 | 각 지도의 `NaN` 셀을 구분할 수 있어야 함 |
| 중복 셀 | 동일 셀은 `wheel > leg > 드론` 순으로 우선하고, 유효값을 `NaN`으로 덮지 않아야 함 |
| 범위 처리 | 서로 다른 지도 범위를 하나의 합집합 출력 범위로 확장해야 함 |
| 시간 | 지도와 병합 결과의 timestamp가 Gazebo `/clock` 기준이어야 함 |
| 저장 | 통합 GridMap을 rosbag2 `mcap` 형식으로 저장해야 함 |
| 네임스페이스 | 입력 지도와 통합 지도 이름이 충돌하지 않아야 함 |
| 메모리 | 출력 지도 범위와 해상도를 적용했을 때 메모리 사용량을 감당할 수 있어야 함 |
| 출력 | 통합 지도 결과를 RViz2에서 표시할 수 있어야 함 |
| 입력 검증 | frame, 레이어 또는 해상도 정보가 없는 지도는 병합하지 않아야 함 |
| 병합 실행 | 병합은 세 완료 상태가 모두 수신된 시점에 1회만 실행되어야 하며, 반복 갱신되지 않아야 함 |

---

## 12. 완료 기준

- **입력 지도 확인:** 세 지도를 읽을 수 있고, frame·해상도·크기·원점·높이 레이어·완료 상태를 확인할 수 있다.
- **공통 좌표 확인:** 세 지도의 frame이 `map`이며, 좌표가 안 맞으면 병합하지 않고 오류를 표시한다.
- **출력 격자 생성 확인:** 세 지도의 범위·해상도를 비교해 합집합 출력 격자의 원점·크기를 결정하고, 메모리 초과 여부를 확인할 수 있다.
- **해상도와 범위 처리 확인:** 해상도가 다르면 리샘플링 없이 병합을 중단하고 오류를 표시하며, 출력 범위가 세 지도의 전체 유효 범위를 포함한다.
- **중복 셀 처리 확인:** `wheel > leg > 드론` 우선순위가 적용되며, 입력 순서와 무관하게 결과가 항상 동일하고, 유효값이 미관측 값으로 덮어써지지 않으며, 이상치가 지도 전체를 왜곡하지 않는다.
- **미관측 셀 처리 확인:** 세 지도 모두 미관측인 영역만 `NaN`이며, `NaN`과 실제 높이 `0m`을 구분할 수 있다.
- **통합 지도 생성 확인:** 세 로봇 관측 영역이 하나의 지도에 `map` 좌표, m 단위, 합의된 해상도로 표시되며 RViz2에서 비교 가능하다.
- **결과 제공 확인:** 통합 지도를 다른 노드와 RViz2가 사용할 수 있고, 병합 완료/오류 여부를 확인할 수 있다. 세 완료 상태가 모두 수신된 시점에 통합 지도가 정확히 1회 생성되며, 이후 입력 지도가 갱신되어도 재병합은 발생하지 않는다.
- **저장·재현 확인:** 통합 GridMap이 `mcap`으로 저장되고, 다시 불러온 뒤에도 프레임·해상도·크기·원점·레이어·미관측 정보·병합 메타데이터가 유지된다.
- **최종 결과 검증:** 드론 지도만으로 안 보이던 지상 세부 영역이 wheel·leg로 보완되고, 세 입력 지도와 통합 지도의 위치 관계가 일치하며, RViz2에서 최종 확인할 수 있다.

---

## 구현 시 참고사항

- 패키지 이름: `agconav_map_fusion`
- ROS2 배포판: Jazzy
- 언어: Python
- 지도 병합 로직(numpy 기반 격자 처리)은 모듈 D의 `ground_elevation_mapper`에서 사용한 방식(grid_map C++ 바인딩 대신 numpy 직접 구현, `grid_map_msgs/msg/GridMap` 메시지로 수동 패킹)을 최대한 재사용한다. 특히 grid_map wire-format 패킹(축 flip, column-major flatten) 규약은 모듈 D와 동일하게 적용해야 통합 지도가 올바르게 렌더링된다.
- 지도 저장 로직(rosbag2 mcap 직렬화)은 모듈 D의 `ground_elevation_map_saver` 구조를 재사용한다.
- `elevation_map` 3종 토픽과 `elevation_map_status` 3종 토픽은 모두 절대 경로(`/drone/...`, `/wheel/...`, `/leg/...`)로 고정 구독한다 (이 노드는 네임스페이스로 재사용되는 다중 인스턴스 구조가 아니라, 단일 통합 노드이므로 CONTRIBUTING 5번의 "상대 토픽 이름" 규칙은 입력 구독 부분에는 적용되지 않음 — 단, 파라미터로 토픽 이름을 노출해 유연성은 유지할 것).
- 병합 트리거는 **정확히 1회만** 발동해야 하며, 이후 재트리거되지 않도록 내부 상태 플래그로 관리할 것.
- 해상도 불일치, frame 불일치 등 검증 실패 시 절대로 임의로 리샘플링하거나 강제로 병합을 진행하지 말고, 명확히 중단하고 오류를 발행할 것.
- 중복 셀 처리는 파이썬 딕셔너리나 set 등 순서가 보장되지 않는 자료구조에 의존하지 말고, 결정적(deterministic) 순서로 처리할 것 (예: wheel 배열 먼저 덮어쓰기 → 그 위에 leg 유효값만 덮어쓰기 → 그 위에 드론 유효값만 덮어쓰기, 벡터화 연산으로 처리).
