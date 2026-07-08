# ugv_husky — 4륜 자율 (Clearpath Husky A300)

**담당:** 이수빈

뚫린 평지에 강한 skid-steer 4륜 로봇. 지상 국소 탐지와 Nav2 주행을 담당한다.

## 범위

- **탐색(지상 탐지):** 주행 중 자기 센서로 목표·장애물·아군 국소 탐지, 드론 가림영역 보완
- Nav2 주행 (costmap + **4륜 traversability 레이어**)
- 통과 실패(막힘/전복 위험) 감지·보고

## 인터페이스(예정)

- 입력: `/map/trav_wheeled`, 목표 리스트
- 출력: 국소 탐지 결과, 통과 실패 이벤트 → orchestration

## 구조(예정)

```
ugv_husky/
├── bringup/        # Husky 스폰·드라이버
├── nav/            # Nav2 파라미터·trav 레이어
├── detect/         # 지상 국소 탐지
└── launch/
```
