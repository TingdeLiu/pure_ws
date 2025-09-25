#!/bin/bash

tmux new-session -d -s ros2_session

# Gazebo
tmux send-keys -t ros2_session "export DISPLAY=:1 && export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms && source /opt/ros/humble/setup.bash && source /home/ubuntu/Rob/pure_ws/install/setup.bash && ros2 launch gazebo_modele gazebo.launch.py" C-m

sleep 5

# Nav2
tmux split-window -v -t ros2_session
tmux send-keys -t ros2_session "source /opt/ros/humble/setup.bash && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true" C-m
sleep 5

# 自动探索
tmux split-window -h -t ros2_session
tmux send-keys -t ros2_session "source /opt/ros/humble/setup.bash && ros2 launch nav_slam 2dpoints_plus.launch.py" C-m

tmux attach -t ros2_session

