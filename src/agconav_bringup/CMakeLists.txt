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
