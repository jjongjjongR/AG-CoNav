# test/ — 실험 하네스와 현황

실험 **결과와 근거**는 `docs/13. 컨트롤러 재실험 — 등판·단차·속도.md` 에 있다.
여기에는 하네스 사용법과, 하네스를 만들며 당한 함정만 남긴다.

## 하네스

```
# leg — 운용 스폰 런치를 무수정으로 쓴다
./test/scripts/run_op.sh <world.sdf> <tag> <kind> <value> <ctl> <policy> <speed> <out.json>
#   ctl: rl | guide | champ,  kind: slope | step | ramp
#   env: INIT_Z(스폰 높이), GAIT_H(guide 발 들어올림), STEER_ARGS
./test/scripts/run_batch.sh 3 1.0     # 4후보 x 3회 등판(단계식 램프)
./test/scripts/run_step.sh  3 1.0     # leg 단차

# wheel — clearpath 스폰
./test/scripts/run_wheel.sh <world.sdf> <tag> <kind> <value> <speed> <out.json>
./test/scripts/run_wheel_sweep.sh 3 0.8   # 경사+단차
./test/scripts/run_wheel_speed.sh 2       # 속도 x 단차 교차

# 월드 생성
python3 test/scripts/make_terrain_worlds.py test/worlds   # 경사/단차 개별
python3 test/scripts/make_ramp_world.py test/worlds/ramp.sdf  # 단계식 5~30도

# 집계
python3 test/scripts/analyze_exp1.py --dir test/results/ramp
python3 test/scripts/analyze_wheel.py --dir test/results/wheel

# 종단 파이프라인 검증 (A->F->B->C->D->E 산출 토픽 확인)
python3 test/scripts/check_pipeline_e2e.py
```

## 반드시 지킬 것

### 1. 고아 프로세스 — 이 세션에서만 다섯 번 당했다

`cleanup.sh` 에 하네스가 띄우는 노드 이름이 빠지면 그 노드가 시행마다
살아남아 다음 시행에 얹힌다. 여러 개가 같은 토픽에 서로 다른 명령을 쏜다.

| 빠뜨린 이름 | 쌓인 수 | 증상 |
|---|---|---|
| `bench_relay` | — | 기립 성공률 50% -> 0% |
| `op_trial` | 4 | 로봇이 초당 2~3 m 튐 |
| `cmd_vel_to_control_input` | **58** | 등판 한계가 10도로 오판(실제 20도) |
| `wheel_trial` | 10 | 같은 토픽 발행자 10개 |
| `twist_mux` 등 clearpath | 16 | wheel 진출 0.00 m |

지금은 P1~P7 로 덮었다. **새 노드를 만들면 반드시 추가할 것.**
실행 전후로 `ps -eo comm --no-headers | sort | uniq -c | sort -rn | head` 확인.

부수 함정 둘:
- `pkill -f` 패턴에 **러너 스크립트 이름**(`run_op`)을 넣으면 배치가 자기
  부모를 죽인다. 죽여야 하는 것은 토픽을 발행하는 노드뿐이다.
- **대화형 셸 명령줄에 그 이름을 쓰면 셸이 자기를 죽인다.** 확인할 때는
  `ps aux | grep "[o]p_trial"` 처럼 대괄호를 쓸 것. heredoc 안에 쓴 문자열도
  `bash -c` 명령줄에 포함되므로 똑같이 걸린다.

### 2. 포즈 계측

- **`gz topic -e` 텍스트를 파이썬으로 파싱하지 말 것.** 링크 17개가 물리
  주기마다 실려 파서가 못 따라가면 값이 몇 초씩 과거가 된다. 서 있을 때는
  멀쩡해 보이다가 걷기 시작하면 위치가 초당 2~3 m 튄다.
- leg: `/leg/odom` (운용 xacro 에 `<dimensions>3</dimensions>` 를 넣어 3D 로 만들었다)
- wheel: 월드 `pose/info` 를 PoseArray 로 브리지하고 **첨자는 셸에서 미리 찾아
  넘긴다**(노드 안에서 찾으면 subprocess 가 스핀을 막아 신호를 놓친다)

### 3. 구동 토픽이 로봇마다 다르다

| | 토픽 | 타입 |
|---|---|---|
| leg | `/leg/cmd_vel` | `Twist` |
| wheel | `/wheel/cmd_vel` | **`TwistStamped`** |

wheel 의 `platform/cmd_vel` 은 twist_mux **출구**다. 거기 직접 쓰면 mux 가
쓰는 0 과 번갈아 덮어써서 로봇이 안 움직인다. 입구로 낼 것.

### 4. clearpath 는 gz_ros2_control 플러그인 경로가 필요하다

`export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:..."` 가 없으면 월드는
뜨는데 하드웨어가 안 붙어 컨트롤러 스포너가 락 대기만 하다 죽는다.

### 5. CHAMP 는 기립 문턱이 다르다

`nominal_height` 0.225 라 RL 기준(0.25)으로 재면 정상 기립을 "기립X" 로
오판한다. `--stand-min-z 0.18` 을 쓴다(`op_trial.py` 가 champ 면 자동 적용).

## 남은 것

- CHAMP 를 단계식 램프로 재측정
- leg 최적 속도 (guide 는 0.4 m/s 가 설계 상한이라 폭이 좁다)
- legged_gym·himloco 가 못 걷는 진짜 이유 (설정은 upstream 과 동일함을 확인)
