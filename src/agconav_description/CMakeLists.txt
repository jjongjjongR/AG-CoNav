cmake_minimum_required(VERSION 3.8)
project(agconav_description)

find_package(ament_cmake REQUIRED)

# 존재하는 디렉터리만 설치한다(빈 패키지 단계에서 빌드가 깨지지 않도록).
foreach(dir models urdf)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
