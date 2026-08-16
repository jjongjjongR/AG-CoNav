cmake_minimum_required(VERSION 3.8)
project(agconav_test_worlds)
find_package(ament_cmake REQUIRED)

# 이 패키지에는 실험 기록(.md)만 남는다.
#
# 실험으로 확정된 것들은 전부 운용 패키지로 옮겼다. 여기 있던 월드·모델·런치·
# 설정을 그대로 두면 "쓰는 것"과 "실험용"이 섞여 어느 쪽이 진짜인지 알 수 없다.
#   정렬 월드      -> agconav_worlds/worlds/Seongdong_gu_aligned
#   전체 스캔 경로 -> agconav_drone/config/scan_path_fullmap.yaml
#   지도 평가 도구 -> agconav_traversability/scripts   (SLAM 비교에 그대로 쓴다)
#   경로 생성기    -> agconav_drone/scripts/make_strip_path.py
#   월드 생성기    -> agconav_worlds/scripts/make_aligned_world.py
#
# 나머지(경사·속도·단차 시험 하네스와 그 전용 월드)는 지웠다. 각 실험이 무엇을
# 어떻게 쟀고 결과가 무엇이었는지는 아래 문서에 남아 있다.
install(
  DIRECTORY ./
  DESTINATION share/${PROJECT_NAME}/docs
  FILES_MATCHING PATTERN "*.md"
)
ament_package()
