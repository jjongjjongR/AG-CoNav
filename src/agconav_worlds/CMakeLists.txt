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
# 정렬 월드(Seongdong_gu_aligned) 생성기. 원본 Seongdong_gu 를 축에 맞춰
# 회전시킨 결과가 그 월드이고, 회전각·중심은 aligned_params.txt 에 남아 있다.
install(
  PROGRAMS scripts/make_aligned_world.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
