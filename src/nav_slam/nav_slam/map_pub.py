#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
障碍物网格图生成节点（ObstacleGridNode）：
- 从点云中识别障碍物
- 对障碍物进行三层膨胀处理，增强路径规划鲁棒性
- 输出 OccupancyGrid 供路径规划器（如 A*、Nav2）使用
- 同时发布话题 /combined_grid 和 /map（Nav2 使用）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class ObstacleGridNode(Node):
    def __init__(self):
        super().__init__('obstacle_grid_node')

        # ---------------- 参数声明与获取 ---------------- #
        self.declare_parameter('grid_width', 60.0)            # 地图宽度（米）
        self.declare_parameter('grid_height', 60.0)           # 地图高度（米）
        self.declare_parameter('resolution', 0.1)             # 栅格地图分辨率（米/格）
        self.declare_parameter('min_height', 0.1)             # 有效点云最小高度（米）
        self.declare_parameter('max_height', 1.0)             # 有效点云最大高度（米）
        self.declare_parameter('obstacle_radius', 0.2)        # 单个障碍物膨胀半径（米）

        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.resolution = self.get_parameter('resolution').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value

        # ---------------- 初始化地图结构 ---------------- #
        self.grid_combined = OccupancyGrid()
        self.grid_combined.header.frame_id = 'map'  # 坐标系为 map
        self.grid_combined.info.width = int(self.grid_width / self.resolution)
        self.grid_combined.info.height = int(self.grid_height / self.resolution)
        self.grid_combined.info.resolution = self.resolution
        self.grid_combined.data = [-1] * (self.grid_combined.info.width * self.grid_combined.info.height)

        # ---------------- ROS 2 通信接口 ---------------- #
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/mapokk', self.pointcloud_callback, 10)  # 点云订阅器
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)                  # 里程计订阅器
        self.grid_combined_pub = self.create_publisher(OccupancyGrid, 'combined_grid', 10)                   # 发布综合地图

        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_profile)  # 给 Nav2 用的 /map 发布器

        # ---------------- 状态变量 ---------------- #
        self.obstacles = set()                    # 原始障碍物点
        self.dilated_obstacles_layer1 = set()     # 第一层膨胀（最靠近障碍）
        self.dilated_obstacles_layer2 = set()     # 第二层膨胀
        self.dilated_obstacles_layer3 = set()     # 第三层膨胀（最远扩展）
        self.odom_data = None                     # 当前里程计数据

    def odom_callback(self, msg):
        self.odom_data = msg  # 保存最新的位姿数据用于地图定位参考

    def pointcloud_callback(self, msg):
        if self.odom_data is None:
            return  # 若无位姿信息，则跳过本次处理

        origin_x = self.odom_data.pose.pose.position.x  # 提取机器人当前坐标作为地图中心
        origin_y = self.odom_data.pose.pose.position.y

        # 解析点云消息（x, y, z）
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        radius_cells = int(self.obstacle_radius / self.resolution)  # 计算障碍半径对应的栅格数

        # 清空临时集合
        new_obstacles = set()
        new_dilated_obstacles_layer1 = set()
        new_dilated_obstacles_layer2 = set()
        new_dilated_obstacles_layer3 = set()

        for x, y, z in points:
            if self.min_height <= z <= self.max_height:
                center_x = int((x + self.grid_width / 2) / self.resolution)
                center_y = int((y + self.grid_height / 2) / self.resolution)

                if 0 <= center_x < self.grid_combined.info.width and 0 <= center_y < self.grid_combined.info.height:
                    index = center_y * self.grid_combined.info.width + center_x
                    new_obstacles.add(index)  # 记录原始障碍

                # 多层障碍物膨胀（3层）
                for layer, dilated_set in enumerate([new_dilated_obstacles_layer1,
                                                     new_dilated_obstacles_layer2,
                                                     new_dilated_obstacles_layer3]):
                    for dx in range(-(layer + 1) * radius_cells, (layer + 1) * radius_cells + 1):
                        for dy in range(-(layer + 1) * radius_cells, (layer + 1) * radius_cells + 1):
                            if dx**2 + dy**2 <= ((layer + 1) * radius_cells)**2:
                                grid_x = center_x + dx
                                grid_y = center_y + dy
                                if 0 <= grid_x < self.grid_combined.info.width and 0 <= grid_y < self.grid_combined.info.height:
                                    index = grid_y * self.grid_combined.info.width + grid_x
                                    dilated_set.add(index)

        # 更新障碍集合
        self.obstacles.update(new_obstacles)
        self.dilated_obstacles_layer1.update(new_dilated_obstacles_layer1)
        self.dilated_obstacles_layer2.update(new_dilated_obstacles_layer2)
        self.dilated_obstacles_layer3.update(new_dilated_obstacles_layer3)

        # 更新地图信息
        self.update_combined_grid()

    def update_combined_grid(self):
        # 重置为通行区域（值为1）
        self.grid_combined.data = [1] * (self.grid_combined.info.width * self.grid_combined.info.height)

        # 标记障碍物（值为100）
        for index in self.obstacles:
            self.grid_combined.data[index] = 100

        # 标记膨胀层（用不同数值表示）
        for index in self.dilated_obstacles_layer1 - self.obstacles:
            if self.grid_combined.data[index] == 1:
                self.grid_combined.data[index] = 5

        for index in self.dilated_obstacles_layer2 - self.dilated_obstacles_layer1:
            if self.grid_combined.data[index] == 1:
                self.grid_combined.data[index] = -8

        for index in self.dilated_obstacles_layer3 - self.dilated_obstacles_layer2:
            if self.grid_combined.data[index] == 1:
                self.grid_combined.data[index] = -120

        # 设置地图原点坐标
        self.grid_combined.header.stamp = self.get_clock().now().to_msg()
        self.grid_combined.header.frame_id = 'map'
        self.grid_combined.info.origin.position.x = -self.grid_width / 2
        self.grid_combined.info.origin.position.y = -self.grid_height / 2
        self.grid_combined.info.origin.position.z = 0.0

        # 发布地图
        self.grid_combined_pub.publish(self.grid_combined)   # 发布到 /combined_grid
        self.map_pub.publish(self.grid_combined)              # 发布到 /map（给Nav2使用）


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGridNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
