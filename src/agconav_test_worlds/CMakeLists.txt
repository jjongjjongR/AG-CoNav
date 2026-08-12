cmake_minimum_required(VERSION 3.8)
project(agconav_test_worlds)
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY worlds launch config models rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/generate_test_world.py
    scripts/generate_scan_path.py
    scripts/velocity_path_follower.py
    scripts/build_map.py
    scripts/eval_map.py
    scripts/feed_cloud.py
    scripts/capture_nav.py
    scripts/send_nav_goals.py
    scripts/check_path_following.py
    scripts/run_traversability.sh
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
