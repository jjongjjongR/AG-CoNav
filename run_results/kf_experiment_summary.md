# Module A 칼만필터 개선 후보 실험 종합 (2026-08-20)

**방법**: baseline #5(칼만필터+GT pose) 코드에서 정확히 하나만 바꾼 변형을
각각 `bags/velocity_4m_5mps_gps_attempt2`(5m AGL/4m/5mps, baseline #5와
동일 조건 bag) 재생(rate=2.0, 0단계에서 baseline #5 대비 오차 <0.3pp로
검증됨)으로 1:1 비교. 매 실험 전 코드를 git checkout으로 baseline
상태로 되돌린 뒤 딱 하나만 수정. 상세 근거는 `run_results/PROGRESS.md`
("[새 세션] Module A 칼만필터 11가지 개선 후보 실험" 절) 참고.

## 기준값
| | wheel FN% | leg FN% | 커버리지 |
|---|---|---|---|
| baseline #5(원 보고, rate=1.0, attempt1 bag) | 10.91% | 8.03% | 95.80% |
| **local baseline(이번 세션, rate=2.0, attempt2 bag)** | **10.68%** | **7.89%** | **95.89%** |

이후 표의 "차이"는 전부 local baseline 기준.

## Phase 1 — 개별 검증 결과

| # | 변형 | wheel FN% | Δwheel | leg FN% | Δleg | 커버리지 | 판정 |
|---|---|---|---|---|---|---|---|
| H | 중앙값 대표값 | 12.55% | +1.87pp | 8.59% | +0.70pp | 95.84% | 기각(악화) |

(진행 중 — 아래 계속 채워짐)

## Phase 2 — 조합 결과
(Phase 1 완료 후 채워짐)

## 최종 추천
(전체 완료 후 채워짐)
