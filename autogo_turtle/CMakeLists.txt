cmake_minimum_required(VERSION 3.8)
project(autogo_turtle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(autogo_turtle_geometry src/autogo_turtle_geometry.cpp)
ament_target_dependencies(
  autogo_turtle_geometry
  "rclcpp"
  "geometry_msgs"
  "turtlesim"
)

add_executable(turtle_pose_sub src/turtle_pose_sub.cpp)
ament_target_dependencies(
  turtle_pose_sub
  "rclcpp"
  "turtlesim"
)

add_executable(autogo_turtle_waypoints src/autogo_turtle_waypoints.cpp)
ament_target_dependencies(
  autogo_turtle_waypoints
  "rclcpp"
  "geometry_msgs"
  "turtlesim"
)

install(TARGETS
  autogo_turtle_geometry
  autogo_turtle_waypoints
  turtle_pose_sub
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
