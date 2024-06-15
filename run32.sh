conda deactivate
colcon build --symlink-install
source install/setup.bash
    # ros2 launch rm_nav_bringup bringup_real.launch.py \
    # world:=RC2024 \
    # mode:=nav \
    # lio:=fastlio \
    # localization:=icp \
    # lio_rviz:=False \
    # nav_rviz:=True
cmds=(
    "ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=RC2024 \
    mode:=nav \
    lio:=fastlio \
    localization:=icp \
    lio_rviz:=False \
    nav_rviz:=True"
	# "ros2 run serial_driver send_move_command"
    # "ros2 run serial_driver action_command"
    "python3 /home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/test.py"
    "ros2 launch serial_driver command.launch.py"
    # "ros2 run serial_driver sub_serial_pub_status"
    "ros2 run rc_decision rc_decision_32"
    # "ros2 run rc_decision game_start"
    # "ros2 run rc_decision read_nav_status"
    # "ros2 run rc_decision read_pid_status"
    "ros2 run rc_decision vision_decision"
    "ros2 run rc_decision pose_controller"
    "ros2 run rc_decision tf_listener"
    # "ros2 run rc_decision goal_id_pub_node"
    # "ros2 run serial_driver pub_velocity"
    # "ros2 run rc_vision sub_stop_quqiu"
    "ros2 run rc_decision send_chioce"
    # "ros2 run rc_decision send_goal"
    # "ros2 run rc_decision param_id_pub_node"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.5
done

