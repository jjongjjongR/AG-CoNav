# agconav_bringup

전체 시스템을 한 번에 실행하는 통합 launch를 위한 패키지.

## Clearpath 설정 경로

저장소의 `config/clearpath_a300/robot.yaml`은 빌드할 때
`share/agconav_bringup/config/clearpath_a300/robot.yaml`로 설치된다.
통합 launch는 ament package share 경로를 사용하므로 사용자 홈 디렉터리,
클론 위치, `--symlink-install` 사용 여부와 관계없이 같은 명령으로 실행된다.

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch agconav_bringup agconav_sim.launch.py
```

별도 Clearpath 설정을 시험할 때만 launch 인자를 재정의한다.

```bash
ros2 launch agconav_bringup agconav_sim.launch.py \
  clearpath_setup_path:=/absolute/path/to/clearpath_a300
```
