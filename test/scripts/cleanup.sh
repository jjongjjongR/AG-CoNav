#!/bin/bash
# 시뮬 잔재 정리. **반드시 스크립트 파일로 부를 것.**
#
# pkill -f 패턴을 대화형 셸에 직접 치면 그 명령줄 자체가 패턴과 일치해
# 셸이 자기를 죽인다(이 프로젝트에서 여러 번 당했다). 스크립트 안에 두면
# 이 프로세스의 명령줄이 "cleanup.sh" 뿐이라 안전하다.
#
# !! 노드 이름을 하나씩 나열하는 방식은 포기했다 !!
# 이 세션에서만 여섯 번 빠뜨렸다. 빠진 노드는 시행이 끝나도 살아남아 다음
# 시행에 그대로 얹히고, 여러 개가 같은 토픽에 서로 다른 명령을 쏜다.
#
#   bench_relay              기립 성공률 50% -> 0%
#   op_trial                 로봇이 초당 2~3 m 씩 튐
#   cmd_vel_to_control_input **58개** 누적, 등판 한계가 10도로 오판
#   wheel_trial              같은 토픽 발행자 10개
#   twist_mux 등 clearpath   구독자 16개, wheel 진출 0.00 m
#   drone_velocity_follower  path_status 를 래치해 새 실행이 6초 만에 "완료" 오판
#
# 마지막에는 **부하 평균 417** 까지 갔다. Nav2 스택이 7~9벌씩 중복 실행돼
# 메모리 20 GB 를 먹고 기계가 멈췄다(static_transform_publisher 20개,
# lifecycle_manager 9개, smoother_server 7개 ...). 모듈 B/C/F 노드와
# nav2_bringup 이 띄우는 서버들이 패턴에 하나도 없었기 때문이다.
#
# 그래서 이름이 아니라 **공통 표식**으로 지운다. ROS 2 노드는 예외 없이
# 명령줄에 `--ros-args` 를 갖는다(런치가 붙여 준다). 이거 하나면 모듈이
# 늘어나도 다시 빠뜨릴 일이 없다.
# !! 패턴을 '--ros-args' 라고 그대로 쓰면 안 된다 !!
# pkill 이 앞의 '--' 를 자기 옵션으로 해석해서 패턴이 통째로 무시된다.
# 실제로 그 상태에서 모듈 F 노드가 72분째 살아남아 있었다.
# 대괄호로 감싸 정규식으로 만들면 pkill 이 패턴으로 받는다.
ROS_MARK='[-][-]ros-args'
GZ1='gz sim'
GZ2='ruby.*gz'
# 런치 런너 자체(자식이 죽어도 부모가 남아 다시 띄우는 것을 막는다).
LAUNCH='ros2 launch|ros2 run|start_pipeline|start_short'
# --ros-args 가 안 붙는 예외들.
EXTRA='ros2 daemon|robot_state_publisher|parameter_bridge'
# !! 이 줄이 7번째 사고를 막는다 !!
# 이 저장소의 파이썬 노드를 `python3 src/...` 로 직접 띄우면 --ros-args 도
# 없고 `ros2 run` 도 아니라 위 패턴을 전부 빠져나간다. 실제로
# replay_elevation_map.py 가 **13개** 살아남아 5초마다 /drone/elevation_map
# 을 계속 밀었고(그중 둘은 아예 다른 지도였다), 새 실행의 모듈 F 가 부팅
# 6초 만에 고아가 준 지도로 판정을 끝내 버렸다. 재생이 55초 뒤에 도착해도
# 판정기는 다시 돌지 않아 실행 전체가 무효가 됐다.
# 경로로 잡는다 — 모듈이 늘어나도 다시 빠뜨리지 않는다.
PYNODE='python3 src/agconav|python3 test/scripts|python3 scripts/'

for p in "$GZ1" "$GZ2" "$LAUNCH" "$ROS_MARK" "$EXTRA" "$PYNODE"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 3
# 남은 것이 있으면 한 번 더(자식이 늦게 정리되는 경우가 있다).
for p in "$ROS_MARK" "$PYNODE"; do pkill -9 -f "$p" 2>/dev/null; done
sleep 1
find /dev/shm -maxdepth 1 \( -name "fastrtps*" -o -name "sem.fastrtps*" \) -delete 2>/dev/null
exit 0
