cmake_minimum_required(VERSION 3.8)
project(agconav_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Clearpath의 robot.yaml은 저장소 공용 설정(config/clearpath_a300)에 있지만,
# 런타임에는 소스 트리의 위치를 추측하지 않고 패키지 share에서 읽는다.
# 생성 산출물(URDF, launch, params)은 clearpath_gz의 generate:=true가
# 실행 중 setup_path 아래에 다시 만들므로 원본 robot.yaml만 설치하면 된다.
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../config/clearpath_a300/robot.yaml
  DESTINATION share/${PROJECT_NAME}/config/clearpath_a300
)

ament_package()
