# 성능진단 리포트 — brian_test 3차 무인 실행 (2026-08-14)

이 세션에서 실행한 1회분(launch 시작 2026-08-14 18:33:10, monitor 종료
19:23:32, FAILED)에 대한 진단. 근거는 모두 `/tmp/agconav_full_run.log`
(479KB, 3166줄)와 `/tmp/agconav_status_log.txt`(71KB, 2087줄, 2분 간격
16회 스냅샷)에서 직접 인용. 추측이 필요한 지점은 명시적으로 "확정 못함"으로
표시.

## 0. 결론 요약 (핵심 원인 2가지)

1. **`merge_wait_timeout_sec=30.0s`가 이 파이프라인의 실제 기동 시간보다
   구조적으로 짧다.** wheel/leg의 Nav2 lifecycle이 "Activating" 단계에
   도달하기까지만도 실측 ~115초가 걸렸고(§3), 실제 waypoint 주행 완료는
   당연히 그보다 훨씬 뒤. `map_merge_collector`의 30초 타임아웃은 Nav2가
   시동도 걸기 전에 이미 만료된다 — 리소스 압박이 전혀 없었어도 이 값
   그대로는 성공 자체가 불가능한 설계/설정 불일치다.
2. **머신 사양(RAM 5.8Gi) 대비 동시 기동 프로세스가 과도해 실행 시작
   2분 이내에 이미 스왑에 진입하고, load average가 실행 내내 30~80을
   오간다(§2).** 이로 인해 (a) wheel의 map→lidar TF가 시뮬레이션 시각
   108.88초 지점에서 완전히 멈춰버리고 이후 전혀 갱신되지 않았고(§4),
   (b) leg의 포인트클라우드 스트림이 실행 내내 간헐적으로 2~6초씩
   끊겼으며(§4), (c) 제 자신의 모니터링 스크립트(`ros2 topic echo`)조차
   47분 동안 이미 발행된 latched 토픽을 읽어내지 못했다(§1) — 관측 자체가
   신뢰할 수 없어졌다.

두 원인은 독립적이다: 1번만 고쳐도(타임아웃을 늘려도) 2번의 리소스
압박이 그대로면 wheel의 TF가 영구히 멈추는 문제 때문에 여전히 실패할
가능성이 높고, 2번만 고쳐도(리소스를 늘려도) 타임아웃이 30초 그대로면
여전히 실패한다. 두 가지 모두 조치가 필요하다.

## 1. 타임라인 재구성 — "47분 대기"는 실제로는 대부분 관측 실패였다

| 시각(wall) | elapsed | 사건 | 출처 |
|---|---|---|---|
| 18:33:10 | 0s | `ros2 launch` 시작 | 이번 세션 기록 |
| 18:33:58 | ~48s | monitor 스크립트 기동 (source 시간 포함) | 이번 세션 기록 |
| ~18:34:53 | ~103s | `map_merge_collector` 내부 상태: wheel/leg 상태변경 없이 30초 경과 시작점(추정, 역산) | grid_math/map_merge_collector.py 로직 |
| **18:35:55** | **~165s** | **`map_merge_collector`가 실제로 merge_wait_timeout 발화, `/merged/merge_error` 발행(latched)** | full_run.log:1103 `merge wait timed out, aborting: ... (still missing: wheel, leg)` |
| 18:36:06 ~ 19:20:59 | 128s ~ 2821s (iteration 1~14) | **모니터의 `ros2 topic echo --once /merged/merge_error`가 14회 연속 실패** (`<none>`, 3초 타임아웃 내 미수신) — 토픽은 latched라 이미 값이 있었는데도 못 읽음 | status_log.txt 각 iteration |
| **19:23:32** | **2974s** | 모니터가 처음으로 성공적으로 `/merged/merge_error`를 읽어 marker 기록, 세션이 이를 감지 | status_log.txt:1956, marker |

즉 **실제 파이프라인의 판정은 launch 시작 후 약 165초 만에 이미 끝나
있었다.** 그 뒤 약 47분은 시스템이 뭔가를 더 시도한 시간이 아니라,
`ros2 topic echo --once`가 매 2분마다 반복적으로 discovery에 실패한
시간이다. `/merged/merge_error`는 `map_merge_collector.py`에서
`QoSDurabilityPolicy.TRANSIENT_LOCAL, depth=1`로 발행되므로(코드 확인,
아래 §5), 정상적인 부하에서는 새 구독자가 언제 붙든 즉시 마지막 값을
받아야 한다. 14번 연속 실패는 §2의 리소스 압박과 시간적으로 정확히
겹친다 — load average가 실행 내내 30~80이었던 구간과 동일하다.

**교훈(다음 세션 참고)**: 앞으로 이런 latched 상태 토픽을 폴링할 때
`ros2 topic echo --once`에 실패하면 "아직 안 왔다"로 해석하면 안 된다.
이 환경에서는 discovery 자체가 부하에 취약해서 "왔는데 못 읽었다"일
가능성이 훨씬 크다. `ros2 topic list`로 토픽 존재 여부와 echo 성공률을
같이 보거나, 가능하면 rosbag record로 latched 토픽을 캡처하는 편이
더 신뢰도 높다.

## 2. 메모리 / CPU — 초반 급등 후 정체(누수 아님)

`free -h` 스냅샷 16개 전체(2분 간격) 요약:

| elapsed | Mem used | Mem free | Swap used | load average(1m) |
|---|---|---|---|---|
| 0s | 3.8Gi | 126Mi | 596Ki | 11.29 |
| 128s | 5.1Gi | 107Mi | 1.3Gi | 70.13 |
| 275s | 5.0Gi | 254Mi | 1.4Gi | 58.89 |
| 420s | 4.9Gi | 125Mi | 1.5Gi | 54.16 |
| 720s | 4.8Gi | 154Mi | 1.8Gi | 70.07 |
| 871s | 4.8Gi | 185Mi | 1.9Gi | 79.34 |
| 1918s | 4.8Gi | 140Mi | 1.9Gi | 63.06 |
| 2374s | 4.6Gi | 182Mi | 2.0Gi | 56.44 |
| 2974s | 4.6Gi | 206Mi | 2.0Gi | 32.13 |

**판정: 지속적 누수 패턴이 아니다.** 실행 시작 후 약 2분 만에 이미
메모리 사용량(4.8~5.1Gi)과 스왑(~1.3~2.0Gi)이 거의 최종 수준까지
도달하고, 그 뒤 49분 내내 거의 그 값 그대로 정체(±0.3Gi 이내)한다.
이는 "특정 노드가 시간이 갈수록 계속 더 먹는다"가 아니라, **애초에
동시 기동되는 프로세스 총량(gzserver + wheel/leg 각각의 전체 Nav2
스택(controller/planner/smoother/bt_navigator/waypoint_follower/
behavior_server/docking_server 등) + robot_localization EKF 4개 +
navsat_transform 2개 + traversability/localization/map_fusion
노드들, 총 66개 이상 프로세스)이 5.8GiB RAM 머신의 물리 한계를 처음부터
초과한다**는 뜻이다. load average도 같은 패턴: 시작 2분 뒤부터 계속
CPU 코어 수 대비 수 배(30~80)를 유지하며 회복되지 않는다 — 특정
구간에서 튄 게 아니라 실행 내내 상시 과부하 상태.

`top -bn1 상위 15줄` 16개 스냅샷에서 1위 CPU 소비 프로세스 집계:
**`ruby` 프로세스가 16회 중 16회 모두 1위** — 이는 이상 프로세스가
아니라 Gazebo 시뮬레이터 본체(`gz sim`, PID 5383)다. Gazebo의 `gz`
CLI 런처가 Ruby로 구현되어 있어 `ps`에는 `ruby3.2`로 표시되는 것으로,
직접 `readlink /proc/5383/exe` 및 cmdline으로 확인함 (정상 동작, 이상
현상 아님). 2~3위는 매 스냅샷마다 `ground_elevation_mapper`,
`drone_path_player`, `parameter_bridge`, `drone_elevation_mapper` 등이
번갈아 나타남 — 하나의 노드가 폭주하는 게 아니라 다수 노드가 골고루
CPU를 나눠 쓰면서 전체적으로 공급이 부족한 그림이다.

## 3. Nav2 lifecycle 기동 자체가 이미 115초 — 리소스 압박이 없어도 30초 타임아웃은 못 맞춘다

`agconav_sim.launch.py`의 `TimerAction` 단계(이번 실행에는 30/60/90초로
조정된 미커밋 버전이 적용됨, PROGRESS.md 참고)에 따라 nav2 그룹은
launch 시작 90초 뒤에야 기동을 시작한다. 로그 실측:

- `waypoint_follower` lifecycle 노드 생성: wheel/leg 모두 elapsed ~60초
  (`Creating`, full_run.log:683,716)
- `Configuring` 단계 진입: leg ~104초, wheel ~105초 (full_run.log:979,981)
- `Activating` 단계 진입: leg ~113초, wheel ~116초 (full_run.log:1083,1096)

즉 **Nav2가 "막 켜지기 시작한" 시점(113~116초)이 이미
`map_merge_collector`의 실제 타임아웃 발화 시점(~165초)에 근접해
있고**, 그마저도 아직 waypoint 주행은 시작도 안 한 상태다(lifecycle
Activating ≠ 주행 완료). `merge_wait_timeout_sec` 파라미터는 소스
주석(`map_merge_collector.py:78-82`)에 스스로 "정확한 근거는 없는
placeholder... 실측/시나리오 기반 재조정 필요"라고 적혀 있는데, 이번
실측으로 그 우려가 정확히 들어맞았다: 30초는 Nav2가 활성화되는 데
걸리는 시간보다도 짧다.

## 4. wheel/leg 개별 원인 — 서로 다른 증상

**wheel: map→lidar TF가 시뮬레이션 108.88초 지점에서 완전히 멈춤.**
`ground_elevation_mapper`가 `wheel/lidar3d_0_sensor_link`→`map` TF를
반복 조회하는데, 최초 몇 차례는 정상이었다가 이후 계속 동일한 에러가
반복된다:

```
[wheel.ground_elevation_mapper] TF lookup failed for
  "wheel/lidar3d_0_sensor_link" -> "map" at 110.100000000s, dropping cloud:
  Lookup would require extrapolation into the future.
  Requested time 110.100000 but the latest data is at time 108.880000
```

이 패턴이 wall-clock 기준 elapsed ~1975초부터 ~3042초까지(약 17분간)
동일한 `108.880000`을 "가장 최신 데이터"로 반복 보고한다
(full_run.log:1510~2526) — **TF 버퍼가 그 이후로 단 한 번도 갱신되지
않았다는 뜻**. 포인트클라우드 자체는 계속 들어오고 있었으므로(그래서
매번 새 타임스탬프로 조회를 시도함) 문제는 포인트클라우드가 아니라
wheel 쪽 로컬라이제이션 체인(ekf_node → navsat_transform_node 또는
robot_state_publisher)이 그 시점 이후로 TF를 더 이상 발행하지 않았다는
것. 정확히 어느 노드가 멈췄는지는 해당 노드들 자체의 로그에 크래시나
에러가 없어(조용히 멈춤) 이번 로그만으로 확정할 수 없음 — CPU 스케줄링
기아(§2의 상시 과부하) 때문에 해당 노드의 타이머/콜백이 그냥 스케줄되지
않았을 가능성이 가장 유력하나 **확정하지 못함**.

**leg: 간헐적 포인트클라우드 스톨(구조적 정지 아님).**
```
[leg.ground_elevation_mapper] no point cloud in the last 2.20s (topic may be stalled).
```
류의 경고가 elapsed ~500초부터 종료 시점까지 총 30회 가까이 반복
관측되나, 매번 2~6초 뒤 회복되어 다시 데이터가 들어온다(완전히 멈추지
않음, wheel과 다름). 이는 `no point cloud received yet`이 아니라
`no point cloud in the last Xs` 형태로, 이미 데이터를 받은 적이 있는
상태에서 일시적으로 끊기는 패턴 — §2의 CPU 경합으로 인한 간헐적
메시지 지연/드롭으로 보임(leg의 point cloud 발행/브릿지 프로세스가
순간적으로 스케줄을 못 받는 경우). 초반(elapsed ~33초)에는 별도로
`leg/os1_lidar -> map ... Tf has two or more unconnected trees` 에러도
1회 관측됐으나(full_run.log:644) 곧바로 사라져 초기 TF 트리 조립
순서 문제로 보이며 지속적 원인은 아닌 것으로 판단.

**drone: 44개 waypoint 로드 완료 후, 실행 종료(SIGINT, elapsed
~2657초)까지 끝내 완료를 보고하지 않음.** `/drone/path_status`가 로그
전체에서 단 한 번도 발행되지 않았고, "waypoint 완료"류 로그도 없음.
드론은 `map_merge_collector`의 트리거 조건에 포함되지 않아(설계상
의도적, 코드 docstring 참고) 이번 실패의 직접 원인은 아니지만, 44개
waypoint를 teleport 방식으로 도는 것조차 50분 안에 못 끝냈다는 것은
"드론은 upfront에 빨리 끝난다"는 설계 전제(map_merge_collector.py의
docstring: "드론이 wheel/leg보다 항상 먼저 끝난다")가 이런 부하
상황에서는 성립하지 않을 수 있음을 시사한다. teleport 자체는 물리
연산이 필요 없는 가벼운 동작이라 §2의 전역 CPU 경합이 여기에도
영향을 준 것으로 보이나, drone_pose_controller/drone_path_player
자체의 대기/재시도 로직(예: `max_retries=3`, 서비스 응답 대기) 때문일
가능성도 배제 못함 — **확정하지 못함**, 코드까지 정독하지는 않았음
(이번 실패의 직접 원인이 아니라 우선순위를 낮춤).

## 5. 알려진 후보 원인 대조

| 후보 | 이번 실행 관측 여부 |
|---|---|
| Gazebo 나무(모델) 72개 원격 다운로드 | **관측 안 됨.** 로그 전체에 Fuel/download 관련 메시지 전무 (`grep -i "fuel\|download"` 결과 없음). 모델이 이미 로컬 캐시돼 있었던 것으로 보임 — 이번엔 이게 원인이 아니었다. |
| 물리 스텝(physics step) 설정 | **특이사항 없음.** world 파일(`Seongdong_gu.world`)의 `<physics name="1ms" type="ignored"><dart>...` 블록에 `max_step_size`/`real_time_factor`가 명시적으로 지정돼 있지 않음 — gz-sim 기본값을 그대로 씀. 유난히 미세한 스텝으로 오버라이드된 상태가 아니었다. |
| 드론 지도 규모(500x500m world) | **간접적으로 관측됨.** §4에서 서술한 대로 드론이 50분 내내 44 waypoint 순회를 못 끝냈다 — 다만 물리 스텝이나 지도 자체 크기 때문인지, 아니면 §2의 전역 리소스 경합 때문인지는 이번 로그만으로 분리 확정 못함. |
| wheel `joint_state_broadcaster` 이중 스폰 경쟁(2차 실행에서 지목된 원인) | **재발 안 함.** 미커밋 상태로 남아있던 TimerAction 마진 조정(15/30/40s → 30/60/90s, `agconav_sim.launch.py`)이 효과가 있었던 것으로 보임 — `spawner-31` 관련 FATAL/"already loaded" 에러가 이번 로그에는 전혀 없고, joint_state_broadcaster가 정상적으로 Loaded→Configured→Activated까지 순조롭게 진행됨(full_run.log:276-285). |
| DDS 공유메모리 포트 충돌 | 이번 로그에서 `RTPS_TRANSPORT_SHM` 관련 에러 미발견 (grep 결과 없음) — 재발 안 함. |
| navsat_transform "more than 20d from N pole" | **재발함**, 그러나 1회성. leg에서 elapsed ~30초 지점 1회만 발생(full_run.log:635) 후 반복되지 않음 — 초기화 과정 중 datum 설정 전 잠깐의 경고로 보이며 지속적 영향은 확인 안 됨. |
| ROS2 discovery 불안정(메모리 압박 하) | **이번에도 확실히 재발, 오히려 이번 세션에서 더 명확히 재현됨** (§1의 47분 오탐 사례). |

## 6. 권장 조치 (코드는 수정하지 않음, 제안만)

1. **`merge_wait_timeout_sec`을 최소 5~10분 이상으로 늘리거나(설계
   placeholder임을 소스 주석이 이미 인정하고 있음), 30초 고정 타임아웃
   대신 "wheel/leg Nav2가 실제로 waypoint 주행을 시작했는지"를 감지해
   그 시점부터 타이머를 재기산하는 방식으로 바꾸는 것을 검토.** 근본
   원인 1번(§3)의 직접 해법.
2. **동시 기동 프로세스 총량을 줄이거나 머신 RAM을 늘리는 것 검토.**
   현재 5.8GiB로는 시작 2분 만에 이미 스왑에 들어간다. 후보: RViz
   비활성화(이미 headless 사용 중), wheel/leg 중 하나만 먼저 완료 후
   순차 실행, 또는 Nav2 스택 중 이번 시나리오에 불필요한 서버
   (docking_server 등)를 조건부로 끄는 것.
3. **wheel의 TF가 조용히 멈추는 문제(§4)는 어느 노드가 멈췄는지부터
   특정해야 함.** 다음 실행 시 `ros2 topic hz /tf` 또는 각 로컬라이제이션
   노드(ekf_node, navsat_transform_node, robot_state_publisher)의
   개별 CPU/스케줄링 상태를 diagnostics로 남기면 특정 가능할 것으로
   보임. 리소스 압박(2번)을 먼저 해소하면 이 증상 자체가 사라질 가능성도
   있음 — 순서상 2번을 먼저 시도해볼 가치가 있음.
4. **모니터링 스크립트가 `ros2 topic echo --once`의 단발 실패를 "미발행"으로
   오판하지 않도록, latched 상태 토픽은 재시도 또는 `ros2 topic list`
   교차확인을 병행하는 방식으로 개선.** (§1) 이번 진단 자체의 방법론
   개선 제안이며 AG-CoNav 소스코드와는 무관.
5. 드론(§4)의 장시간 미완료는 이번 실패의 직접 원인이 아니므로 후순위지만,
   2번 조치 이후에도 계속되면 `drone_pose_controller`/`drone_path_player`
   자체의 재시도/대기 파라미터를 별도로 점검할 필요가 있음.

## 7. Ground truth 비교 (4번 단계)

**수행하지 않음.** `merge_status`가 True로 발행된 적이 없어(§0, §1)
지시사항에 따라 스킵. merged map 자체가 생성되지 않았으므로 비교할
대상이 없음.
