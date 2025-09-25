# 基于ROS2 Humble的自主探索仿真平台

本项目复现并对比了两种基于前沿点检测的自主环境探索算法：
1.  **传统RRT探索算法**：使用RRT（快速扩展随机树）进行全局路径规划，结合A*和DWA（动态窗口法）进行局部路径规划和避障。
2.  **改进的RRT探索算法 (HFD)**：一种混合前沿点检测策略，结合了可变步长RRT、多根节点RRT和基于网格的检测算法，同样结合A*和DWA进行路径规划。

仿真环境基于ROS2 Humble、Gazebo和Nav2构建。

## 核心技术

*   **仿真平台**: ROS2 Humble, Gazebo
*   **导航与定位**: Nav2, SLAM Toolbox
*   
*   **探索算法**: Python
*   **路径规划**: A* (全局), DWA (局部)
*   **容器化**: Docker

## 算法简述

### 1. 传统RRT探索
该方法使用单个RRT树来探索未知空间。通过在地图上随机采样生成节点，并从树中寻找最近的节点进行扩展，逐步构建一棵探索树。当前沿点被发现后，机器人使用A*算法规划一条全局路径，并由DWA控制器执行局部导航。

### 2. 改进的RRT探索 (HFD)
该方法源于论文 `Hybrid_Frontier_Detection_Strategy_for_Autonomous_Exploration_in_Multi-obstacles_Environment`，其核心思想是结合多种前沿点探测策略的优点：
*   **混合前沿点检测**: 结合了可变步长随机树、多根节点随机树和基于网格的算法，以提高在复杂和多障碍物环境中的探测效率和覆盖率。
*   **目标点优化**: 使用均值漂移聚类算法（Mean Shift）对探测到的前沿点进行聚类，并从中选出最优前沿点作为机器人的下一个导航目标。
*   **路径规划**: 同样采用A*算法进行全局路径规划和DWA算法进行局部路径规划。

这种混合策略旨在克服单一RRT方法在狭窄通道或复杂障碍物区域容易陷入局部最优的问题。

## 环境搭建与配置 (Docker)

本项目强烈建议使用Docker进行环境搭建，以避免复杂的依赖配置。

### 1. 先决条件
*   安装 [Docker](https://www.docker.com/get-started)
*   安装支持VNC的客户端（如 [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/)），用于查看仿真图形界面。

### 2. 构建Docker镜像
在项目根目录下，执行以下命令构建镜像：
```bash
docker build -t ros2_exploration_env -f docker/Dockerfile.vnc .
```

### 3. 启动Docker容器
执行以下命令启动容器。这会将当前项目目录挂载到容器的 `/root/ws_pure` 路径下，并映射VNC服务的端口 `5901`。
```bash
docker run -it --rm --name exploration_sim \
  -p 5901:5901 \
  -v "$(pwd):/root/ws_pure" \
  ros2_exploration_env
```
*注意：在Windows上，请将 `$(pwd)` 替换为 `${pwd}` (PowerShell) 或 `%cd%` (CMD)。*

### 4. 编译工作空间
在容器的shell中，执行以下命令编译ROS2工作空间：
```bash
cd /root/ws_pure
colcon build
```

### 5. 启动仿真
编译成功后，在容器内运行启动脚本来开始仿真。脚本使用 `tmux` 在后台启动所有必要的节点。
```bash
# 启动改进的RRT (HFD) 算法仿真
# 注意：脚本内默认启动的是改进版算法，如需切换，请修改 `start_exploration.sh`
bash docker/start_exploration.sh
```
启动后，您可以通过VNC客户端连接到 `localhost:5901` (密码: `123456`) 来查看Gazebo和RViz的图形化界面。

## 实验结果对比

我们在多个地图环境中对两种算法进行了测试，以评估其探索效率。

---

### **地图1 (map1)**

| 算法 | 探索总用时 (秒) | 总路径长度 (米) | 路径快照 | 探索视频 |
| :--- | :---: | :---: | :---: | :---: |
| **传统RRT** | 0.50 | 149.45 | [查看图片](./Demo/map1/rrt/rrt_path_snapshot.png) | [观看视频](./Demo/map1/rrt/vokoscreenNG-2025-06-01_17-27-24.mkv) |
| **改进RRT (HFD)** | 0.00 | 155.98 | [查看图片](./Demo/map1/rrt_plus/rrt_exploration.png) | [观看视频](./Demo/map1/rrt_plus/vokoscreenNG-2025-06-01_17-36-42.mkv) |

---

### **地图4 (map4)**

| 算法 | 探索总用时 (秒) | 总路径长度 (米) | 路径快照 | 探索视频 |
| :--- | :---: | :---: | :---: | :---: |
| **传统RRT** | 0.00 | 159.33 | [查看图片](./Demo/map4/rrt/rrt_exploration.png) | [观看视频](./Demo/map4/rrt/vokoscreenNG-2025-06-01_18-05-15.mkv) |
| **改进RRT (HFD)** | 0.72 | 165.52 | [查看图片](./Demo/map4/rrt_plus/rrt_path_snapshot.png) | [观看视频](./Demo/map4/rrt_plus/vokoscreenNG-2025-06-01_17-56-42.mkv) |

---

### **地图5 (map5)**

| 算法 | 探索总用时 (秒) | 总路径长度 (米) | 路径快照 | 探索视频 |
| :--- | :---: | :---: | :---: | :---: |
| **传统RRT** | 0.02 | 173.58 | [查看图片](./Demo/map5/rrt/rrt_path_snapshot.png) | [观看视频](./Demo/map5/rrt/vokoscreenNG-2025-06-01_18-19-41.mkv) |
| **改进RRT (HFD)** | 0.00 | 176.21 | [查看图片](./Demo/map5/rrt_plus/rrt_exploration.png) | [观看视频](./Demo/map5/rrt_plus/vokoscreenNG-2025-06-01_18-26-58.mkv) |

---

### **地图6 (map6)**

| 算法 | 探索总用时 (秒) | 总路径长度 (米) | 路径快照 | 探索视频 |
| :--- | :---: | :---: | :---: | :---: |
| **传统RRT** | 0.92 | 139.95 | [查看图片](./Demo/map6/rrt/rrt_path_snapshot.png) | [观看视频](./Demo/map6/rrt/room6_rrt.mkv) |
| **改进RRT (HFD)** | 0.00 | 143.91 | [查看图片](./Demo/map6/rrt_plus/rrt_exploration.png) | [观看视频](./Demo/map6/rrt_plus/rrt_plus.mkv) |

*注：路径长度由轨迹点累加计算得出，可能与实际里程计读数略有差异。探索用时为算法节点运行时间，不完全代表总仿真时间。*

## 参考文献
1.  Ma, F., Liu, T., & Zhang, Q. (2021). **A Hybrid Frontier Detection Strategy for Autonomous Exploration in Multi-obstacles Environment**. *2021 40th Chinese Control Conference (CCC)*.
2.  Zou, X., Wang, P., & Liu, D. (2013). **Autonomous robotic exploration based on multiple rapidly-exploring randomized trees**. *2013 IEEE International Conference on Robotics and Biomimetics (ROBIO)*.
