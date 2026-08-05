cmake_minimum_required(VERSION 3.8)
project(agconav_worlds)
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
