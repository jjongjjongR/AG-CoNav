# quad_go2 — 4족 자율 (Unitree Go2 + CHAMP)

**담당:** 채현우 / 이수빈

잔해·계단 같은 험지에 강한 4족 로봇. CHAMP 보행으로 학습 없이 즉시 구동하며, 지상 탐지와 Nav2 상위 경로를 담당한다.

## 범위

- CHAMP 보행 통합 (`unitree_go2_ros2_jazzy`)
- **탐색(지상 탐지):** rough/narrow 구간에서 국소 탐지
- Nav2 상위 경로 (**4족 traversability 레이어**)
- 계단·턱 접근 시퀀스, 통과 실패 감지·보고

## 인터페이스(예정)

- 입력: `/map/trav_legged`, 목표 리스트
- 출력: 국소 탐지 결과, 통과 실패 이벤트 → orchestration

## 확장

동일 Go2 모델이 MuJoCo Menagerie에 있어 [`quad_rl`](../quad_rl)의 RL 보행으로 자연 확장/대체 가능.

## 구조(예정)

```
quad_go2/
├── bringup/        # Go2 스폰 + CHAMP
├── nav/            # Nav2 + 4족 trav 레이어
├── detect/         # 지상 국소 탐지
├── gaits/          # 계단·턱 접근 시퀀스
└── launch/
```
