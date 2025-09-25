#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# 简化但完整的 DWA 控制器 + 路径跟踪 ROS 2 节点（含中文注释）

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import Imu
import math
import numpy as np
from scipy.spatial import KDTree
import yaml
import os

# 动态窗口法控制器类（Dynamic Window Approach）
class DWAController:
    def __init__(self, config):
        # 初始化控制参数
        self.dwa_controller = DWAController({
            'max_vel': 0.3,            # 最大线速度（单位：m/s）
            'min_vel': 0.0,            # 最小线速度（可为 0）
            'max_omega': 1.0,          # 最大角速度（单位：rad/s）
            'min_omega': -1.0,         # 最小角速度（负值表示可反向转）
            'v_res': 0.05,             # 线速度分辨率（搜索粒度）
            'omega_res': 0.1,          # 角速度分辨率
            'predict_time': 1.0,       # 轨迹预测时间（秒）
            'dt': 0.1,                 # 控制周期或仿真时间步长（秒）

            'cost_weights': {          # 各类代价的权重设置（可根据表现调整）
                'to_goal': 1.0,        # 朝向目标的代价（越小越好）
                'obstacle': 1.0,       # 离障碍物距离的代价（越远越好）
                'velocity': 1.0        # 线速度代价（鼓励快走）
            },

            'obstacle_threshold': 50   # 大于该值视为障碍（默认地图值为0-100）
        })

        # self.max_vel = config.get('max_vel', 0.5)             # 最大线速度
        # self.min_vel = config.get('min_vel', 0.0)             # 最小线速度
        # self.max_omega = config.get('max_omega', 1.0)         # 最大角速度
        # self.min_omega = config.get('min_omega', -1.0)        # 最小角速度
        # self.v_res = config.get('v_res', 0.05)                # 线速度分辨率
        # self.omega_res = config.get('omega_res', 0.1)         # 角速度分辨率
        # self.predict_time = config.get('predict_time', 1.0)   # 轨迹预测时间
        # self.dt = config.get('dt', 0.1)                       # 时间步长
        # self.cost_weights = config.get('cost_weights', {
        #     'to_goal': 1.0,        # 目标代价
        #     'obstacle': 1.0,      # 障碍代价
        #     'velocity': 1.0       # 速度代价
        # })
        # self.obstacle_threshold = config.get('obstacle_threshold', 50)  # 障碍值阈值

    # 轨迹预测函数
    def predict_trajectory(self, pose, v, w):
        x, y, theta = pose
        traj = []
        for _ in np.arange(0, self.predict_time, self.dt):
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            traj.append((x, y))
        return traj

    # 计算到目标的距离代价
    def calc_to_goal_cost(self, traj, goal):
        last_x, last_y = traj[-1]
        dx = goal[0] - last_x
        dy = goal[1] - last_y
        return math.hypot(dx, dy)

    # 计算速度代价
    def calc_velocity_cost(self, v):
        return self.max_vel - v

    # 计算轨迹上所有点的障碍物代价
    def calc_obstacle_cost(self, traj, costmap, resolution, origin):
        cost = 0.0
        for x, y in traj:
            gx = int((x - origin[0]) / resolution)
            gy = int((y - origin[1]) / resolution)
            if 0 <= gx < costmap.shape[1] and 0 <= gy < costmap.shape[0]:
                val = costmap[gy, gx]
                if val >= self.obstacle_threshold:
                    return float('inf')  # 碰撞，代价为无穷大
                cost += val
        return cost / len(traj)

    # 主控制命令生成函数，返回最优的速度 (v, w)
    def compute_cmd(self, pose, goal, costmap, resolution, origin):
        angle_diff = math.atan2(goal[1] - pose[1], goal[0] - pose[0]) - pose[2]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # 面向目标偏差大，先原地转向
        if abs(angle_diff) > math.radians(90):
            return 0.0, 0.5 * np.sign(angle_diff)

        best_score = float('inf')
        best_v, best_w = 0.0, 0.0

        # 遍历所有可能的速度组合，计算其代价
        for v in np.arange(self.min_vel, self.max_vel + self.v_res, self.v_res):
            for w in np.arange(self.min_omega, self.max_omega + self.omega_res, self.omega_res):
                traj = self.predict_trajectory(pose, v, w)
                to_goal_cost = self.calc_to_goal_cost(traj, goal)
                obstacle_cost = self.calc_obstacle_cost(traj, costmap, resolution, origin)
                velocity_cost = self.calc_velocity_cost(v)
                total_cost = (
                    self.cost_weights['to_goal'] * to_goal_cost +
                    self.cost_weights['obstacle'] * obstacle_cost +
                    self.cost_weights['velocity'] * velocity_cost
                )
                if total_cost < best_score:
                    best_score = total_cost
                    best_v, best_w = v, w

        return best_v, best_w

# ROS 2 路径跟踪节点
class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following_node')
        self.dwa_controller = DWAController({
            'max_vel': 0.3,
            'min_vel': 0.0,
            'max_omega': 1.0,
            'min_omega': -1.0,
            'v_res': 0.05,
            'omega_res': 0.1,
            'predict_time': 1.0,
            'dt': 0.1,
            'cost_weights': {
                'to_goal': 1.0,
                'obstacle': 1.0,
                'velocity': 1.0
            },
            'obstacle_threshold': 50
        })

        self.path_points = None
        self.current_odom = None
        self.path_received = False
        self.stop_flag = False
        self.occupancy_data = None
        self.resolution = None
        self.origin = None

        # 创建 ROS 2 订阅者与发布者
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/path', self.path_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/map', self.costmap_callback, 10)

    # 路径消息回调函数
    def path_callback(self, msg):
        self.path_points_list = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self.path_points = np.array(self.path_points_list)
        assert self.path_points.ndim == 2
        self.path_points = self.interpolate_path(self.path_points)
        self.path_received = True

    # 插值路径点，提高控制平滑度
    def interpolate_path(self, points, segment_length=0.1):
        interpolated_points = []
        for i in range(len(points) - 1):
            start_point, end_point = points[i], points[i+1]
            distance = np.linalg.norm(end_point - start_point)
            num_points = int(distance / segment_length) + 1
            t_values = np.linspace(0, 1, num_points)
            segment = start_point + (end_point - start_point)[np.newaxis, :] * t_values[:, np.newaxis]
            interpolated_points.append(segment)
        return np.vstack(interpolated_points)

    # 四元数转偏航角（yaw）
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    # 里程计回调函数，进行路径跟踪控制
    def odometry_callback(self, msg):
        if not self.path_received:
            return
        self.current_odom = msg
        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, self.quaternion_to_yaw(msg.pose.pose.orientation)]
        dist_to_end = np.linalg.norm(np.array(pose[:2]) - self.path_points[-1])
        if dist_to_end < 0.2:
            self.path_received = False
            v, w = 0.0, 0.0
        else:
            if self.occupancy_data is None or self.resolution is None or self.origin is None:
                return
            position = np.array(pose[:2])
            distances = np.linalg.norm(self.path_points - position, axis=1)
            goal = None
            for offset in range(5, 20):
                idx = min(len(self.path_points) - 1, np.argmin(distances) + offset)
                x, y = self.path_points[idx]
                col = int((x - self.origin[0]) / self.resolution)
                row = int((y - self.origin[1]) / self.resolution)
                if 0 <= row < self.occupancy_data.shape[0] and 0 <= col < self.occupancy_data.shape[1]:
                    if self.occupancy_data[row, col] < 80:
                        goal = (x, y)
                        break
            if goal is None:
                self.get_logger().warn("No valid goal found in path ahead. Skipping this cycle.")
                return
            v, w = self.dwa_controller.compute_cmd(pose, goal, self.occupancy_data, self.resolution, self.origin)
        cmd = Twist()
        cmd.linear.x = max(0.0, min(self.dwa_controller.max_vel, v))
        cmd.angular.z = max(-self.dwa_controller.max_omega, min(self.dwa_controller.max_omega, w))
        self.cmd_vel_publisher.publish(cmd)

    # 地图信息回调，提取代价地图数据
    def costmap_callback(self, msg):
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.occupancy_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

# 程序入口点

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()