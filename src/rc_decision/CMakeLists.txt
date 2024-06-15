cmake_minimum_required(VERSION 3.8)
project(rc_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# add_executable(pub_goal_pose src/pub_goal_pose.cpp)
# ament_target_dependencies(pub_goal_pose rclcpp geometry_msgs std_msgs nav2_msgs)  # Add nav2_msgs here
add_executable(send_goal src/send_goal.cpp)
ament_target_dependencies(send_goal rclcpp geometry_msgs nav2_msgs rclcpp_action)

add_executable(param_setter src/param_setter.cpp)  
ament_target_dependencies(param_setter rclcpp std_msgs rcl_interfaces)  
add_executable(read_nav_status src/read_nav_status.cpp)  
ament_target_dependencies(read_nav_status rclcpp std_msgs action_msgs)  

add_executable(goal_id_pub_node src/goal_id_pub_node.cpp)
ament_target_dependencies(goal_id_pub_node rclcpp std_msgs action_msgs)
add_executable(position_id_pub src/position_id_pub.cpp)
ament_target_dependencies(position_id_pub rclcpp std_msgs action_msgs)
add_executable(param_id_pub_node src/param_id_pub_node.cpp)
ament_target_dependencies(param_id_pub_node rclcpp std_msgs)
add_executable(move_mode_pub_node src/move_mode_pub_node.cpp)
ament_target_dependencies(move_mode_pub_node rclcpp std_msgs)
add_executable(action_mode_pub_node src/action_mode_pub_node.cpp)
ament_target_dependencies(action_mode_pub_node rclcpp std_msgs)

add_executable(robot_mode_publisher src/robot_mode_publisher.cpp)
ament_target_dependencies(robot_mode_publisher rclcpp std_msgs)

add_executable(nav_control src/nav_control.cpp)
ament_target_dependencies(nav_control rclcpp geometry_msgs std_msgs)
add_executable(rc_decision src/rc_decision.cpp)
ament_target_dependencies(rc_decision rclcpp geometry_msgs std_msgs)
add_executable(rc_decision_32 src/rc_decision_32.cpp)
ament_target_dependencies(rc_decision_32 rclcpp geometry_msgs std_msgs)
add_executable(game_start src/game_start.cpp)
ament_target_dependencies(game_start rclcpp std_msgs)
add_executable(vision_decision src/vision_decision.cpp)
ament_target_dependencies(vision_decision rclcpp std_msgs geometry_msgs )
add_executable(send_chioce src/send_chioce.cpp)
ament_target_dependencies(send_chioce rclcpp std_msgs)

add_executable(param_adjuster src/param_adjuster.cpp)
ament_target_dependencies(param_adjuster rclcpp std_msgs rcl_interfaces)

add_executable(tf_listener src/tf_listener.cpp)
ament_target_dependencies(tf_listener rclcpp tf2_ros geometry_msgs tf2_eigen geometry_msgs)

add_executable(pose_controller src/pose_controller.cpp)
ament_target_dependencies(pose_controller rclcpp geometry_msgs tf2 tf2_geometry_msgs std_msgs)

install(TARGETS
  # pub_goal_pose
  send_goal
  param_setter
  read_nav_status
  goal_id_pub_node
  param_id_pub_node
  move_mode_pub_node
  action_mode_pub_node
  robot_mode_publisher
  game_start
  nav_control
  rc_decision
  rc_decision_32
  vision_decision
  send_chioce
  param_adjuster
  tf_listener
  pose_controller
  position_id_pub
  DESTINATION lib/${PROJECT_NAME}
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
