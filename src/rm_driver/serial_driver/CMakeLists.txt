cmake_minimum_required(VERSION 3.8)
project(serial_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(pub_velocity src/pub_velocity.cpp)
ament_target_dependencies(pub_velocity rclcpp rclcpp_action geometry_msgs nav2_msgs std_msgs serial)
add_executable(pub_odom src/pub_odom.cpp)
ament_target_dependencies(pub_odom rclcpp nav_msgs serial)
add_executable(pub_target src/pub_target.cpp)
ament_target_dependencies(pub_target rclcpp std_msgs serial)
add_executable(send_move_command src/send_move_command.cpp)
ament_target_dependencies(send_move_command rclcpp std_msgs geometry_msgs serial)

add_executable(sub_serial_pub_status src/sub_serial_pub_status.cpp)
ament_target_dependencies(sub_serial_pub_status rclcpp std_msgs serial)
add_executable(action_command src/action_command.cpp)
ament_target_dependencies(action_command rclcpp std_msgs serial)


install(TARGETS #添加目标的安装路径
  pub_velocity
  pub_target
  pub_odom
  send_move_command
  action_command
  sub_serial_pub_status
  DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY launch
	    DESTINATION share/${PROJECT_NAME})

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
