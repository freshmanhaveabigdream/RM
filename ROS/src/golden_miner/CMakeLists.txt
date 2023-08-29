cmake_minimum_required(VERSION 3.8)
project(golden_miner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mine_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
add_executable(miner src/miner.cpp)
ament_target_dependencies(miner rclcpp mine_interfaces)
install(TARGETS
  miner
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(gold_mine src/gold_mine.cpp)
ament_target_dependencies(gold_mine rclcpp mine_interfaces)
install(TARGETS
  gold_mine
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(visualization src/path.cpp)
ament_target_dependencies(visualization rclcpp mine_interfaces nav_msgs visualization_msgs geometry_msgs)
install(TARGETS
  visualization
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)