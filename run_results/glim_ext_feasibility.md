# glim_ext 빌드 가능성 조사 (0단계)

**결론: 성공.** 이 VM(4코어/5.8GB, aarch64, ROS2 Jazzy)에서 `koide3/glim_ext`를
빌드했고, 별도 워크스페이스를 소싱하면 GLIM이 확장 모듈(`libimu_validator.so`
등)을 정상 인식·로드하는 것까지 실측으로 확인했다.

## 0-1. 빌드

```
mkdir -p ~/glim_ext_ws/src && cd ~/glim_ext_ws/src
git clone https://github.com/koide3/glim_ext
cd ~/glim_ext_ws && source /opt/ros/jazzy/setup.bash && colcon build
```

첫 시도부터 별도 조치 없이 성공(`Summary: 1 package finished`). 별도의
`-dev` 패키지 추가 설치는 필요 없었다 — GTSAM/Boost/OpenCV/OpenMP 등
필요한 라이브러리가 이미 이 환경(apt PPA로 GLIM을 설치하며 함께 들어온
`gtsam_points`/GTSAM 개발 헤더 등)에 갖춰져 있었다.

빌드된 확장 모듈(.so) 중 성공적으로 컴파일된 것:
- `libglim_ext.so` (코어 유틸리티)
- `libgravity_estimator.so`
- `liblogger_hook_demo.so`, `libglim_callback_demo.so` (예제)
- `libimu_prediction.so`
- **`libimu_validator.so`** — 이번 진단 ④에서 쓰려던 바로 그 모듈, 정상 빌드됨
- **`libgnss_global.so`** — GPS 사후결합 대신 검토했던 네이티브 GNSS 모듈, 정상 빌드됨
- `libdeskewer.so`, `libvelocity_suppressor.so` (모듈 목록엔 있으나 개별 확인은 안 함)

**빌드 안 된 것(서브모듈 미획득으로 자동 제외, 에러 아님)**:
- `libodometry_estimation_fastlio.so` (FAST-LIO2 래퍼) — `thirdparty/FAST_LIO`
  서브모듈 URL이 `git@github.com:koide3/FAST_LIO`(SSH 전용)라 익명 clone이
  안 됨(`git submodule status`에서 `-`(uninitialized) 확인). CMakeLists.txt가
  서브모듈 부재를 감지하면 해당 타겟만 조용히 건너뛰는 구조라 전체 빌드는
  실패하지 않았다.
- `orb_slam_frontend`, `dbow_loop_detector`, `scan_context_loop_detector`도
  동일 사유(서브모듈 미획득)로 건너뜀 — 이번 진단(④ LiDAR-IMU 외부보정)엔
  불필요해 추가 조치 안 함.

## 0-2. GLIM의 인식 여부 확인

**1차 시도(글로벌 환경만, glim_ext 워크스페이스 미소싱)** — GitHub 이슈 #9와
정확히 동일한 실패를 재현:
```
[glim] [warning] glim_ext package path was not found!!
[glim] [info] load libimu_validator.so
[glim] [warning] failed to open libimu_validator.so
[glim] [warning] libimu_validator.so: cannot open shared object file: No such file or directory
[glim] [error] failed to load libimu_validator.so
```

**2차 시도(`source ~/glim_ext_ws/install/setup.bash` 추가)** — 정상 로드:
```
[glim] [warning] Extension modules are enabled!!
[glim] [warning] You must carefully check and follow the licenses of ext modules
[glim] [info] config_ext_path: /home/hyunwoo-chae/glim_ext_ws/install/glim_ext/share/glim_ext/config
[glim] [info] load libimu_validator.so
```
(에러/경고 없이 로드 완료. "not found" 문제는 apt로 설치한 GLIM 자체의
문제가 아니라 순수하게 glim_ext 워크스페이스를 소싱 안 한 환경 설정
문제였다.)

**결론**: `glim_ext`는 이 환경에서 완전히 사용 가능하다. 앞으로 GLIM을
실행할 때 `source ~/glim_ext_ws/install/setup.bash`를 (메인 워크스페이스
소싱 뒤에) 추가하기만 하면 `libimu_validator.so`/`libgnss_global.so`를
config의 `extension_modules`에 등록해 쓸 수 있다.

## 라이선스 확인

- glim_ext 저장소 자체: `package.xml` `<license>GPLv3</license>`. 최상위
  `LICENSE` 파일은 없음(각 모듈이 package.xml로 라이선스 명시).
- README 상단 경고문(원문 그대로): *"This repository constains half-baked
  code that may not be well-maintained and not suitable for practical
  purposes."*, *"We don't have resource to maintain this extension
  library."* — "closed-source"라는 표현은 아니고, "실용 목적에 안 맞을 수
  있는 미완성 코드"라는 유지보수 경고다.
- README Disclaimer: *"Each module in glim_ext uses several external
  libraries that employ different licensing conditions. You must
  carefully check and follow their licenses."*
- **`gnss_global`**: 자체 `package.xml`에 `<license>GPLv3</license>` 명시,
  외부 서드파티 의존성 없음(코드 자체가 factor graph 제약 구현). 라이선스
  이슈 없이 사용 가능해 보인다(GPLv3 조건 하에).
- **FAST-LIO2 래퍼(`fastlio2/`)**: 별도 `package.xml` 없음(상위 glim_ext
  패키지에 포함되므로 상위 GPLv3 적용). 단, 감싸는 대상인
  `thirdparty/FAST_LIO` 서브모듈(SSH 전용 URL이라 이번엔 못 받아옴, 원본은
  HKU-Mars/FAST_LIO 기반)의 라이선스는 이번 세션에서 직접 열어보지
  못했다 — **실제로 켜려면 다음 세션에서 SSH 접근 또는 HTTPS 미러로
  서브모듈을 별도 확보해 라이선스 파일을 직접 확인해야 한다.**
- 참고로 `scan_context_loop_detector`는 CC BY-NC-SA 4.0(비상업), `DBoW3`는
  별도 LICENSE.txt — 이번 진단(④)엔 사용 안 함.

**이번 진단 범위(④, imu_validator)에는 라이선스 문제가 없다** — GPLv3이고
비상업 제한 조항도 없다. gnss_global도 같은 조건. FAST-LIO2만 서브모듈을
못 받아 실물 확인이 안 됐다(이번 세션에서 켜지도 않았음, 지시사항대로).

## 다음 세션을 위한 참고

- glim_ext 워크스페이스: `~/glim_ext_ws` (src/glim_ext, build/, install/) —
  그대로 남겨둠, 재빌드 불필요.
- GLIM을 glim_ext 확장 모듈과 함께 실행하려면:
  ```
  source /opt/ros/jazzy/setup.bash
  source ~/glim_ext_ws/install/setup.bash
  ros2 run glim_ros glim_rosnode --ros-args -p config_path:=<config dir>
  ```
  이때 config의 `config_ros.json` → `extension_modules`에 원하는 .so
  파일명을 추가하면 된다.
