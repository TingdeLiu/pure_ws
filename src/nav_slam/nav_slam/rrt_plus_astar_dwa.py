
# 改进版本：集成 Hybrid Frontier Detection (HFD) + Mean Shift + A* + DWA
# 参考论文: Hybrid Frontier Detection Strategy for Autonomous Exploration
# 支持多根RRT、多策略前沿探测器和均值漂移聚类
# 原作者代码基础上修改：实现自主目标点选取与路径规划

import rclpy
from rclpy.node import Node
import numpy as np
import math
import heapq
import random
import json
import os
import matplotlib.pyplot as plt
import cv2
import time
from sklearn.cluster import MeanShift
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker
from scipy.interpolate import BSpline


def costmap(data, width, height, resolution):
    data = np.array(data).reshape(height, width)
    wall_mask = data == 100
    expansion_size = 5  # 膨胀5格，= 0.25米
    inflation_value = 100  # 设置为50，不是致命障碍
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            shifted_mask = np.roll(wall_mask, (i, j), axis=(0, 1))
            data[shifted_mask] = np.maximum(data[shifted_mask], inflation_value)
    data = data * resolution
    return data


def detect_frontiers(grid):
    frontiers = []
    height, width = grid.shape
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if grid[y, x] == 0:
                neighbors = [grid[y + dy, x + dx] for dx in [-1, 0, 1] for dy in [-1, 0, 1]]
                if -1 in neighbors:  # 未探索区域
                    frontiers.append((x, y))
    return frontiers

def is_reachable(start, goal, grid, max_steps=50):
    from collections import deque
    visited = set()
    queue = deque()
    queue.append((start, 0))
    while queue:
        current, steps = queue.popleft()
        if steps > max_steps:
            break
        if current == goal:
            return True
        for d in [(1,0), (-1,0), (0,1), (0,-1)]:
            n = (current[0]+d[0], current[1]+d[1])
            if (0 <= n[0] < grid.shape[0] and 0 <= n[1] < grid.shape[1] and
                grid[n] < 50 and n not in visited):
                visited.add(n)
                queue.append((n, steps + 1))
    return False

def mean_shift_cluster(self, points):
    if len(points) < 2:
        return points
    clustering = MeanShift(bandwidth=3.0).fit(points)
    centers = clustering.cluster_centers_
    valid_centers = []

    robot_row = int((self.y - self.origin[1]) / self.resolution)
    robot_col = int((self.x - self.origin[0]) / self.resolution)
    start = (robot_row, robot_col)

    for cx, cy in centers:
        row = int((cy - self.origin[1]) / self.resolution)
        col = int((cx - self.origin[0]) / self.resolution)
        if 0 <= row < self.height and 0 <= col < self.width:
            if self.occupancy_data[row, col] < 50:
                # ✅ 加入路径验证
                if is_reachable(start, (row, col), self.occupancy_data):
                    valid_centers.append((cx, cy))
    return valid_centers


def astar(start, goal, grid):
    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))
    came_from = {}
    cost_so_far = {start: 0}
    closed_set = set()

    print(f"A* start: {start}, val={grid[start[0], start[1]]}")
    print(f"A* goal: {goal}, val={grid[goal[0], goal[1]]}")

    # 起点终点有效性判断
    if grid[start[0], start[1]] >= 60 or grid[start[0], start[1]] < 0:
        print("Start in obstacle or unknown.")
        return []
    if grid[goal[0], goal[1]] >= 60 or grid[goal[0], goal[1]] < 0:
        print("Goal in obstacle or unknown.")
        return []

    while open_set:
        _, current_cost, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        if current in closed_set:
            continue
        closed_set.add(current)
        for d in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
            neighbor = (current[0] + d[0], current[1] + d[1])
            if (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and
                grid[neighbor] >= 0 and grid[neighbor] < 60):
                new_cost = cost_so_far[current] + grid[neighbor]
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    heapq.heappush(open_set, (priority, new_cost, neighbor))
                    came_from[neighbor] = current
    return []

def bezier_smoothing(array, num_points):
    try:
        array = np.array(array)
        x, y = array[:, 0], array[:, 1]
        dx = np.diff(x, prepend=x[0])
        dy = np.diff(y, prepend=y[0])
        chord_lengths = np.sqrt(dx**2 + dy**2)
        t = np.concatenate(([0], np.cumsum(chord_lengths)))
        t /= t[-1]
        k = num_points - 1
        t_knots = np.concatenate(([0]*k, t, [1]*k))
        x_padded = np.pad(x, (k, k), 'edge')
        y_padded = np.pad(y, (k, k), 'edge')
        spline_x = BSpline(t_knots, x_padded, k, extrapolate=False)
        spline_y = BSpline(t_knots, y_padded, k, extrapolate=False)
        t_new = np.linspace(0, 1, num_points)
        x_smoothed = spline_x(t_new)
        y_smoothed = spline_y(t_new)
        return np.column_stack((x_smoothed, y_smoothed))
    except:
        return array


# 后续将补充完整 NavigationControl 类与 main 函数部分

class NavigationControl(Node):
    def __init__(self):
        super().__init__('navigation_hfd')
        self.map_subscription = self.create_subscription(OccupancyGrid, 'combined_grid', self.map_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, '/target_point', self.goal_callback, 10)
        self.path_publisher = self.create_publisher(Path, 'path2', 10)
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.marker_pub = self.create_publisher(Marker, '/target_marker', 10)
        self.json_path = os.path.expanduser('~/rrt_summary.json')
        self.img_path = os.path.expanduser('~/rrt_exploration.png')
        
        self.occupancy_data = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None
        self.grid = None
        self.goal = None

        self.x = 0.0
        self.y = 0.0
        self.start_time = None
        self.end_time = None
        self.track = []

        self.in_motion = False
        self.frames_dir = '/tmp/rrt_frames/'
        os.makedirs(self.frames_dir, exist_ok=True)
        self.frame_count = 0


    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"目标接收: {self.goal}")
        self.send_marker(self.goal)

        # 发布 RViz Marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE  # 或 ARROW, CYLINDER, CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal[0]
        marker.pose.position.y = self.goal[1]
        marker.pose.position.z = 0.1
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0  # 永久显示
        self.marker_pub.publish(marker)



    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
  
        # ✅ 记录轨迹点（去重，间隔0.2米）
        if not self.track or math.hypot(self.track[-1][0] - self.x, self.track[-1][1] - self.y) > 0.2:
            self.track.append([round(self.x, 3), round(self.y, 3)])


    def send_marker(self, goal):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal[0]
        marker.pose.position.y = goal[1]
        marker.pose.position.z = 0.1
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        self.marker_pub.publish(marker)
    


    def map_callback(self, msg):
        if self.goal is None:
            return
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.width = msg.info.width
        self.height = msg.info.height
        self.occupancy_data = costmap(msg.data, self.width, self.height, self.resolution)

        robot_row = int((self.y - self.origin[1]) / self.resolution)
        robot_col = int((self.x - self.origin[0]) / self.resolution)
        goal_row = int((self.goal[1] - self.origin[1]) / self.resolution)
        goal_col = int((self.goal[0] - self.origin[0]) / self.resolution)

        # ✅ 越界检查
        if not (0 <= robot_row < self.height and 0 <= robot_col < self.width):
            self.get_logger().warn("机器人位置超出地图边界")
            return
        if not (0 <= goal_row < self.height and 0 <= goal_col < self.width):
            self.get_logger().warn("目标点位置超出地图边界")
            return
        grid = np.copy(self.occupancy_data)
        grid[(grid >= -2) & (grid <= 5)] = 1
        grid[(grid < -2) | (grid > 5)] = 100 

        # ✅ 起点自动修复（避免站在膨胀障碍物上）
        def find_nearest_free(grid, x, y, max_radius=5):
            for r in range(1, max_radius + 1):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
                            if grid[nx, ny] < 60:
                                return (nx, ny)
            return (x, y)

        robot_row, robot_col = find_nearest_free(grid, robot_row, robot_col)



        start = (robot_row, robot_col)
        goal = (goal_row, goal_col)

        path_raw = astar(start, goal, grid)
        if not path_raw:  # 如果路径为空
            self.get_logger().warn("A* 找不到路径")
            return
        path_coords = [(p[1] * self.resolution + self.origin[0], p[0] * self.resolution + self.origin[1]) for p in path_raw]
        path_smooth = bezier_smoothing(path_coords, len(path_coords))
        # 强制插入目标点为最后一个点（确保精确对齐）
        if self.goal is not None:
            goal_x, goal_y = self.goal
            if len(path_smooth) == 0 or math.hypot(path_smooth[-1][0] - goal_x, path_smooth[-1][1] - goal_y) > 0.02:
                path_smooth = np.vstack([path_smooth, [goal_x, goal_y]])
        self.send_path(path_smooth)

    def save_final_trajectory_plot(self):
        if self.occupancy_data is None or len(self.track) < 2:
            self.get_logger().warn("⚠️ 无法保存轨迹图：无地图或轨迹数据")
            return

        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(self.occupancy_data, cmap='gray', origin='lower')

        # 将轨迹点从世界坐标转换为地图像素坐标
        track_np = np.array(self.track)
        px = (track_np[:, 0] - self.origin[0]) / self.resolution
        py = (track_np[:, 1] - self.origin[1]) / self.resolution
        ax.plot(px, py, 'r-', linewidth=2, label='Trajectory')

        # 标记起点与终点
        ax.plot(px[0], py[0], 'go', label='Start', markersize=8)
        ax.plot(px[-1], py[-1], 'bo', label='End', markersize=8)

        ax.set_title("Final Exploration Trajectory")
        ax.legend()
        plt.grid(True)
        plt.savefig(self.img_path)
        plt.close()
        self.get_logger().info(f"✅ 最终轨迹图已保存到：{self.img_path}")

    def send_path(self, coords):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in coords:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)
        self.path_publisher.publish(path_msg)

        if not self.follow_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 follow_path server 不可用")
            return

        goal = FollowPath.Goal()
        goal.path = path_msg
        self.start_time = time.time()
        self.follow_path_client.send_goal_async(goal)
        self.get_logger().info("导航路径已发送")

        # ✅ 保存轨迹总结为 JSON
        self.end_time = time.time()
        duration = round(self.end_time - self.start_time, 2) if self.start_time else None

        summary = {
            'start_time': self.start_time,
            'end_time': self.end_time,
            'duration_seconds': duration,
            'trajectory': self.track
        }
        with open(self.json_path, 'w') as f:
            json.dump(summary, f, indent=2)
        self.get_logger().info(f"轨迹总结已保存到：{self.json_path}")
        self.save_final_trajectory_plot()




def main(args=None):
    rclpy.init(args=args)
    node = NavigationControl()
    rclpy.spin(node)
    rclpy.shutdown()