cmake_minimum_required(VERSION 3.8)
project(agconav_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Clearpath 설정은 저장소 공용 위치(config/clearpath_a300)에 있지만,
# 런타임에는 소스 트리를 추측하지 않고 패키지 share에서 읽는다.
# spawn은 generate:=false로 실행되므로 생성 산출물(robot.urdf.xacro, 서비스
# launch, 파라미터)을 런타임에 다시 만들지 않는다. 따라서 robot.yaml만 설치하면
# 나머지가 낡은 채로 남아 소스 수정이 반영되지 않는다(LiDAR 스펙 변경 등).
# 디렉터리 전체를 설치한다.
#
# !! 생성 산출물을 저장소에 넣어 두는 데는 대가가 있다 !!
# clearpath 생성기는 platform/launch/platform-service.launch.py 의 setup_path 에
# **생성한 PC 의 절대경로**를 박아 넣는다. 그대로 커밋하면 다른 PC 에서 없는
# 경로를 가리키는데, setup_simulation.sh 는 그 파일이 있으면 부트스트랩을
# 건너뛰므로 스스로 고쳐지지도 않는다. 그래서 그 한 줄을 FindPackageShare 로
# 바꿔 두었다. robot.yaml 을 고쳐 다시 생성하면 그 수정이 덮이니 확인할 것.
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../config/clearpath_a300
  DESTINATION share/${PROJECT_NAME}/config
)

# launch에서 쓰는 헬퍼 스크립트(월드 로드 대기)는 소스 트리를 추측하지 않고
# 패키지 share에서 실행한다.
install(
  PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/wait_for_world.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
