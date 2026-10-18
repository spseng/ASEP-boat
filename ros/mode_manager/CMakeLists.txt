cmake_minimum_required(VERSION 3.8)
project(mode_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(boat_defs REQUIRED)
find_package(wirelink REQUIRED)

add_executable(mode_manager src/mode_manager.cpp)
target_compile_features(mode_manager PUBLIC cxx_std_17)
target_link_libraries(mode_manager PUBLIC rclcpp::rclcpp ${msgs_TARGETS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${boat_defs_TARGETS} ${wirelink_TARGETS})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
