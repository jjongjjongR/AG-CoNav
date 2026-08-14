# 무인 전체 시스템 실행 결과 요약

## 1. 실행 시각 / 소요 시간

| 항목 | 값 |
|---|---|
| 작업 시작 (git checkout 등) | 2026-08-14 14:38:07 |
| `ros2 launch` 시작 | 2026-08-14 14:38:54 |
| launch 프로세스 완전 종료 | 2026-08-14 14:39:21 (launch 시작 후 약 27초 만에 자체적으로 전체 종료) |
| 정리/보고 작업 종료 | 2026-08-14 14:41:55 |
| **launch 자체 소요 시간** | **약 27초** (2시간 대기 루프까지 갈 필요 없이 launch 시작 직후 전체 시스템이 스스로 종료됨) |

저장소 루트: `/home/hyunwoo-chae/AG-CoNav` (pwd로 확인)
브랜치: `brian_test` (test_main → brian_test 전환, 원래 working tree는 clean하여 stash 불필요)
빌드: `colcon build --symlink-install` 성공 (15개 패키지, exit code 0, deprecation 경고만 존재)

## 2. 로봇별(wheel/leg/drone) 완료 상태

**세 로봇 모두 완료하지 못함.** 모든 로봇 노드가 시작된 지 약 14초 만에(월드 로드 완료 직후) 강제 종료됨.

| 로봇 | 상태 | 비고 |
|---|---|---|
| wheel | 미완료 (강제 종료) | `ground_elevation_mapper`/`ground_elevation_map_saver`가 기동은 했으나 `navigation_status`/`elevation_map_status`를 발행하기 전에 SIGINT/SIGTERM으로 종료됨 |
| leg | 미완료 (강제 종료) | wheel과 동일 |
| drone | 미완료 (강제 종료) | `drone_elevation_mapper`가 "waypoint 44개 로드 완료"까지 진행했으나 경로 주행 전에 종료됨. `/drone/path_status` 발행 안 됨 |

## 3. merge_status 최종 결과: **실패 (FAILURE)**

`/merged/merge_status`, `/merged/merge_error` 모두 이번 실행에서 **발행되지 않음** (토픽 자체가 등장하지 않음). `elevation_map_merger` 노드는 `merge_trigger`를 기다리는 상태에서 died.

### 실패 원인 (2가지, 독립적으로 발견됨)

**(1) 근본 원인 — 하드코딩된 절대경로 (실제로 이번 실행을 중단시킨 원인)**

world 로드가 완료되고 로봇 spawn이 시작되자마자 다음 예외로 launch 전체가 즉시 종료됨:

```
[ERROR] [launch]: Caught exception in launch: executed command failed.
Command: /opt/ros/jazzy/bin/xacro /home/lee/projects/AG-CoNav/config/clearpath_a300/robot.urdf.xacro
  is_sim:=true gazebo_controllers:=/home/lee/projects/AG-CoNav/config/clearpath_a300/platform/config/control.yaml ...
FileNotFoundError: [Errno 2] No such file or directory:
  '/home/lee/projects/AG-CoNav/config/clearpath_a300/robot.urdf.xacro'
```

wheel 로봇의 `robot_state_publisher`(xacro 처리)가 다른 개발자 계정(`/home/lee/...`)의 절대경로를 참조하고 있어, 현재 머신(`hyunwoo-chae`, 저장소 경로 `/home/hyunwoo-chae/AG-CoNav`)에서는 파일을 찾지 못함. launch_ros의 기본 동작상 하나의 프로세스라도 예외로 죽으면 전체 launch tree가 연쇄적으로 SIGINT → SIGTERM → (일부는) SIGKILL로 종료되며, 그 결과 gazebo, map_merge_collector, elevation_map_merger, merged_elevation_map_saver를 포함한 20개 이상의 노드가 모두 함께 죽음.

이는 이번 작업 범위 밖의 기존 소스/설정 파일에 있는 버그이며, 지시사항에 따라 수정하지 않고 관찰/보고만 함.

**(2) 별도로 확인된 잠재적 문제 — 기존 merged map으로 인한 merge 스킵**

`src/agconav_map_fusion/agconav_map_fusion/elevation_map_merger.py` 코드를 보면, 노드 시작 시 `maps/merged_elevation_map`이 디스크에 이미 존재하면(`_merge_done = os.path.exists(...)`) 이번 실행의 `merge_trigger`를 완전히 무시하도록 되어 있음. 실제로 `maps/merged_elevation_map/`에는 2026-08-06 13:23에 생성된 이전 실행 결과가 이미 존재했고, 이번 실행 로그에도 다음 경고가 찍힘:

```
[WARN] [elevation_map_merger]: merged map already exists at "maps/merged_elevation_map"
  -- assuming a previous run already completed the merge (e.g. this node restarted),
  ignoring merge_trigger.
```

즉 xacro 경로 문제가 없었다 하더라도, 이번 실행에서는 merge가 실제로 다시 수행되지 않고 이전 결과를 그대로 둔 채 스킵되었을 것으로 보임. `maps/merged_elevation_map`은 이번 작업이 생성한 산출물이 아니므로 안전 규칙에 따라 삭제하지 않았음.

## 4. 죽은 노드 목록

`wait_for_world`, `drone_path_player`, `drone_pose_controller`, `static_transform_publisher`를 제외한 사실상 전체 노드가 cascade shutdown으로 종료됨. `process has died` 로그가 확인된 노드:

- `gazebo-1`
- `drone_elevation_mapper-7`
- `elevation_map_saver-8` (drone)
- `ground_elevation_mapper-9` (wheel)
- `ground_elevation_map_saver-10` (wheel)
- `ground_elevation_mapper-11` (leg)
- `ground_elevation_map_saver-12` (leg)
- `map_merge_collector-13`
- `elevation_map_merger-14`
- `merged_elevation_map_saver-15`
- `parameter_bridge-24` (drone_cmd_vel_bridge, exit code -6)

마지막 에러 로그(직접 원인): 위 3절 (1)의 `FileNotFoundError` (xacro, wheel robot_state_publisher).

정리 과정에서 `gz sim` 서버 프로세스(PID 21057)가 launch의 SIGKILL 이후에도 reap되지 않고 남아있는 것을 확인하여 별도로 `kill -9`로 강제 종료함. 그 외 잔여 `ros2`/`gz`/`agconav` 프로세스는 없음을 확인.

## 5. 메모리 / 디스크 사용량

실행 자체가 27초 만에 종료되어 유의미한 리소스 누적은 없었음. 정리 직후 스냅샷:

```
Mem:  total 5.8Gi / used 1.7Gi / free 909Mi / available 4.1Gi
Swap: total 4.0Gi / used 624Ki
Disk(~): 78G total, 28G used, 47G available (38%)
```

## 6. 지도 시각화 / 저장 결과

**해당 없음.** merge_status가 True로 발행된 적이 없으므로 지시사항에 따라 시각화 단계(4번)를 수행하지 않았음. 저장된 지도 이미지 없음.

## 7. 커밋 / push

**수행하지 않음.** 지도 이미지가 생성되지 않았으므로 (지시사항 5번 조건: "지도 이미지가 생성됐을 때만") git add/commit/push를 하지 않음. 현재 커밋 해시는 이번 작업 시작 시점과 동일함 (변경 없음).

## 결론 및 권장 조치 (1차 실행 시점 기준)

1. `robot.urdf.xacro` 등 wheel(clearpath_a300) 관련 launch/config에 `/home/lee/projects/AG-CoNav/...` 절대경로가 하드코딩되어 있어, 다른 사용자/머신에서는 무조건 실패함. 상대경로 또는 `FindPackageShare`/환경변수 기반 경로로 바꾸는 것을 검토 필요.
2. `elevation_map_merger`는 `maps/merged_elevation_map`이 이미 존재하면 이번 실행의 merge를 완전히 스킵하도록 설계되어 있음(의도된 "once-ever" 동작으로 보임 — design.md 참조). 매 실행마다 새로 merge 결과를 보고 싶다면, 실행 전에 `maps/merged_elevation_map`을 정리하는 절차가 필요함(이번 무인 작업에서는 안전 규칙상 임의로 삭제하지 않음).

---

# 2차 실행 (재실행) — xacro 경로 수정 + 기존 merge 결과 삭제 후

1차 실행에서 발견된 두 가지 문제를 사용자 확인/승인 후 조치하고 동일한 절차로 재실행함.

## 사전 조치

- **xacro 하드코딩 경로 수정**: `config/clearpath_a300/platform/launch/platform-service.launch.py`의 `setup_path` 인자를 `/home/lee/projects/AG-CoNav/config/clearpath_a300` 리터럴에서 `PathJoinSubstitution([FindPackageShare('agconav_bringup'), 'config', 'clearpath_a300'])`로 변경 (`--symlink-install`이라 재빌드 없이 즉시 반영, 문법 검사 통과). **git에는 아직 커밋하지 않은 로컬 변경 상태.**
- **기존 merge 결과 삭제**: 사용자 승인 하에 `maps/merged_elevation_map/`(2026-08-06 생성분) 삭제. `maps/wheel_elevation_map`, `maps/leg_elevation_map`은 동일 가드가 없어 그대로 둠.

## 실행 시각

| 항목 | 값 |
|---|---|
| `ros2 launch` 시작 | 2026-08-14 16:37:33 |
| merge_error 수신 (모니터링 종료) | 2026-08-14 16:46:52 |
| 정리(프로세스 종료) 완료 | 2026-08-14 16:48경 |
| **launch 소요 시간** | **약 9분 19초** (1차 실행의 27초보다 훨씬 오래 진행 — xacro 크래시는 해결됨을 확인) |

## 결과: **실패 (FAILURE)**, 그러나 원인은 1차와 다름

xacro 문제는 재발하지 않음 (wheel `robot_state_publisher` 정상 초기화, spawn 성공 로그 확인). 대신 **새로운 문제**로 실패:

```
/merged/merge_status => data: false
/merged/merge_error  => data: 'merge_wait_timeout_sec=30.0s elapsed without both
                         wheel/leg elevation_map_status=True (still missing: wheel, leg)'
```

`map_merge_collector`가 wheel+leg의 `elevation_map_status=True`를 30초 안에 받지 못해 타임아웃, merge_error를 발행하며 정상 종료(노드 자체는 죽지 않음).

### 로그에서 확인된 관련 정황 (근본원인 미확정, 참고용)

1. **wheel `joint_state_broadcaster` 이중 스폰 충돌**: `spawner-31`이 "A controller named 'joint_state_broadcaster' was already loaded inside the controller manager"로 FATAL, exit code 1로 죽음. `agconav_sim.launch.py`에 이미 주석으로 남아있는 `platform_velocity_controller`와 동일한 유형의 "clearpath 스택과 수동 spawner가 같은 컨트롤러를 동시에 로드하려는" 경쟁 상태로 보이나, `joint_state_broadcaster`는 그 조치 대상에서 빠져 있었음.
2. **wheel의 TF lookup 실패로 포인트클라우드 드롭**: `[wheel.ground_elevation_mapper] TF lookup failed for "wheel/lidar3d_0_sensor_link" -> "map" ... dropping cloud: Lookup would require extrapolation into the future` — wheel의 라이다 데이터가 elevation map에 누적되지 못함.
3. **leg `navsat_transform_node`가 반복적으로 ERROR**: `Latitude 37.5437d more than 20d from N pole`가 수 초 간격으로 계속 발생 (leg의 GPS/EKF 체인 관련 경고로 보이나 정확한 영향은 미확인).
4. **Nav2 `map_saver` 서버 하트비트 실패**: `lifecycle_manager_map_saver: CRITICAL FAILURE ... not receiving a heartbeat for 4000 ms`.
5. **DDS 공유메모리 포트 충돌**: `RTPS_TRANSPORT_SHM Error: Failed init_port fastrtps_port7073` — 다수 노드 동시 기동 시 발생.
6. **시스템 메모리 압박**: launch 중 RAM 5.8GiB 중 여유 100~250MiB 수준까지 소진, 스왑 1.3GiB까지 사용. `ros2 node list` 등 CLI 명령이 이 시점에 노드를 못 찾는 오탐을 일으킬 정도로 discovery가 불안정했음 (실제 프로세스는 살아있었음, 모니터링 스크립트에서 `ps -p`로 재확인 후 정정).

1~5가 서로 독립적인 원인인지, 6번 리소스 압박이 공통 원인으로 1~5를 유발한 것인지는 이번 조사로는 확정하지 못함. `joint_state_broadcaster` 경쟁 상태(1번)가 유력한 직접 원인 후보.

## 정리

launch/gz 프로세스 모두 정상 종료 확인 (PID 22994, 23013, 23015). 지도 시각화·커밋/push는 merge_status=True가 아니므로 미수행.

## 다음 단계 제안 (참고용, 미실행)

- `agconav_sim.launch.py`의 `spawn_wheel`에서 clearpath 스택이 `joint_state_broadcaster`도 자체적으로 로드하는지 확인하고, 중복 spawner를 `platform_velocity_controller`처럼 제거하거나 조건부로 만들기.
- 메모리 여유가 매우 적은 상태(스왑 진입)이므로, 동시 기동 노드 수를 줄이거나(예: RViz/불필요 모듈 비활성화) 머신 RAM을 늘리는 것을 검토.
- `merge_wait_timeout_sec`(현재 30.0s)을 늘려서 일시적 지연을 흡수할 여지가 있는지 확인 (다만 근본 원인 해결이 우선).

---

# 3차 실행 (별도 세션, 2026-08-14 18:33~19:24) — joint_state_broadcaster 마진 수정 반영 후

이전 세션 종료 시점에 로컬에 미커밋 상태로 남아있던 `agconav_sim.launch.py`의
TimerAction 마진 조정(15/30/40s → 30/60/90s, 2차 실행에서 지목된
`joint_state_broadcaster` 이중 스폰 경쟁 상태에 대한 대응)을 stash하지
않고 그대로 유지한 채 재실행. 상세 판단 근거는 `run_results/PROGRESS.md`
참고.

## 실행 시각

| 항목 | 값 |
|---|---|
| `colcon build --symlink-install` | 성공 (15개 패키지, deprecation 경고만) |
| `ros2 launch` 시작 | 2026-08-14 18:33:10 |
| `/merged/merge_error` 실제 발행 시각(로그 기준) | 2026-08-14 18:35:55 (elapsed ~165초) |
| 모니터가 결과를 감지한 시각 | 2026-08-14 19:23:32 (elapsed 2974초) — **47분의 간극은 실제 대기가 아니라 모니터의 관측 실패, 아래 참고** |
| launch 정상 종료(SIGINT) | 19:24 경, 이후 고아 `gz sim` 프로세스 1개도 별도 종료 |

## 결과: **실패 (FAILURE)**, 그러나 이전 세션들과는 다른 결론

`joint_state_broadcaster` 이중 스폰 문제는 **재발하지 않음** — 미커밋
상태였던 TimerAction 마진 조정이 효과가 있었던 것으로 확인됨
(FATAL/"already loaded" 에러 전무, joint_state_broadcaster가 순조롭게
Loaded→Configured→Activated). 대신 2차 실행과 동일한 `merge_error`
(`merge_wait_timeout_sec=30.0s elapsed without both wheel/leg
elevation_map_status=True`)로 실패했으나, 이번엔 전체 로그를 정밀
분석해 다음을 확인함:

1. **`merge_wait_timeout_sec=30.0s` 자체가 이 파이프라인엔 구조적으로
   너무 짧다.** wheel/leg의 Nav2 lifecycle이 "Activating" 단계에
   도달하는 데만도 실측 ~115초가 걸렸다(로그 근거 확보) — 즉 리소스
   압박이 전혀 없었어도 Nav2가 채 켜지기도 전에 30초 타임아웃이 먼저
   만료되는 설계/설정 불일치. 소스 코드(`map_merge_collector.py`)
   주석에도 이 값이 "정확한 근거 없는 placeholder"라고 스스로 적혀있음.
2. **머신 RAM(5.8GiB) 대비 동시 기동 프로세스(66개 이상)가 과도해
   실행 시작 2분 만에 이미 스왑 진입, load average 30~80이 실행 내내
   지속.** 이로 인해 wheel의 map→lidar TF가 시뮬레이션 108.88초
   지점에서 완전히 멈춰 이후 전혀 갱신되지 않았고, leg의
   포인트클라우드는 실행 내내 간헐적으로 2~6초씩 끊김.
3. **모니터링 자체의 신뢰도 문제**: `/merged/merge_error`는 실제로는
   실행 165초 시점에 이미 발행됐고(latched QoS 확인), 그 뒤 47분
   동안 모니터의 `ros2 topic echo --once`가 14회 연속으로 이미
   발행된 값을 읽어내지 못했다 — "47분을 기다렸다"가 아니라 "결과는
   3분 만에 났는데 관측이 47분 걸렸다"가 정확한 설명. 시스템 부하
   때문에 ROS2 discovery 자체가 불안정했던 것으로 보임(2차 실행에서도
   같은 현상이 지목된 바 있어 재현성 있음).

상세 분석·근거·권장 조치는 `run_results/performance_diagnosis_20260814_213012.md` 참고.

## 알려진 후보 원인 대조 (이번 실행 기준)

| 후보 | 결과 |
|---|---|
| Gazebo 나무 모델 72개 원격 다운로드 | 관측 안 됨 (이번엔 원인 아님) |
| 물리 스텝(physics step) 설정 | 특이사항 없음 (world 파일에 명시적 오버라이드 없이 기본값 사용) |
| 드론 지도 규모 | 간접 관측(드론이 50분 내내 44 waypoint 순회 미완료) — 원인 분리 확정은 못함 |
| joint_state_broadcaster 이중 스폰 | **재발 안 함** (이전 수정으로 해결 확인) |
| DDS 공유메모리 포트 충돌 | 재발 안 함 |
| ROS2 discovery 불안정(메모리 압박 하) | **재발, 이번 세션에서 더 명확히 재현됨** |

## Ground truth 비교

수행하지 않음 (merge_status=True 발행된 적 없음).

## 정리 / push

launch(PID 5376) SIGINT로 정상 종료 확인, 고아 `gz sim`(PID 5383, 실행
파일은 Gazebo의 Ruby 기반 `gz` CLI라 `ps`상 `ruby`로 표시 — 정상)도
별도 SIGINT로 종료. 최종 잔여 ros2/gz/agconav 프로세스 없음, 메모리
가용 4.3Gi로 정상 회복. `run_results/`만 커밋 대상(소스 수정은 이번
세션 범위 밖이라 미커밋 상태 그대로 유지, 위 판단 근거는
`run_results/PROGRESS.md` 참고).
