cmake_minimum_required(VERSION 3.8)
project(nimbus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(router_interfaces REQUIRED)

# Install Python modules
ament_python_install_package(nimbus)

# Install Python executables
install(PROGRAMS
  nimbus/goal_finder.py
  nimbus/nimbus_navigator.py
  nimbus/navigator.py
  nimbus/robot_finder.py
  nimbus/route_planner.py
  DESTINATION lib/${PROJECT_NAME}
)


# C++ nodes
add_executable(pilot src/pilot.cpp)
target_include_directories(pilot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pilot rclcpp rclcpp_action sensor_msgs nav_msgs mavros_msgs router_interfaces)

install(TARGETS pilot
  DESTINATION lib/${PROJECT_NAME}
)


# Install other stuff
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}
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
