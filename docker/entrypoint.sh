#!/bin/bash
set -e

# 启动虚拟 X 桌面
Xvfb :1 -screen 0 1280x720x24 &
export DISPLAY=:1

# 启动桌面环境（fluxbox）
fluxbox &

# 启动 VNC server
x11vnc -display :1 -nopw -forever -shared -bg -quiet
#x11vnc -forever -usepw -display :1 -shared -rfbport 5900 &
# 启动 noVNC 网页服务
websockify --web=/usr/share/novnc 80 localhost:5900 &

# 加载 ROS 环境
source /opt/ros/humble/setup.bash
source /home/ubuntu/Rob/pure_ws/install/setup.bash

# 启动仿真（可选）
#ros2 launch gazebo_modele gazebo.launch.py

# ✅ 打开一个终端窗口方便操作
xterm & 

# 启动仿真或其他命令
exec /start_exploration.sh
# 这里的 exec 确保脚本结束时不会退出容器



