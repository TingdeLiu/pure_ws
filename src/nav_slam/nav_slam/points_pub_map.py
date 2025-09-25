#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
点云坐标变换节点（PointCloudTransformNode）
- 接收原始点云 /points_raw
- 结合 /odom 中的位姿数据，将点云转换至 map 坐标系下
- 发布变换后的点云至 /mapokk
用途：融合运动信息实现全局点云映射（如构建局部地图或前端感知）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudTransformNode(Node):
    def __init__(self):
        super().__init__('pointcloud_transform_node')

        # ---------------- 参数与发布器设置 ---------------- #
        self.declare_parameter('frame_id', 'map')  # 目标坐标系 ID（默认为 map）

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/points_raw', self.pointcloud_callback, 10)

        self.transformed_pointcloud_pub = self.create_publisher(PointCloud2, '/mapokk', 10)  # 发布转换后的点云

        self.odom_data = None             # 最近的里程计数据
        self.rotation_matrix = None       # 位姿对应的 4x4 变换矩阵

    def odom_callback(self, msg):
        # 提取四元数和位移信息，构造变换矩阵
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        translation = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

        # 计算旋转矩阵（由四元数推导）
        qx, qy, qz, qw = quaternion
        sqx, sqy, sqz = qx*qx, qy*qy, qz*qz

        m00 = 1 - 2 * (sqy + sqz)
        m01 = 2 * (qx*qy - qw*qz)
        m02 = 2 * (qx*qz + qw*qy)
        m10 = 2 * (qx*qy + qw*qz)
        m11 = 1 - 2 * (sqx + sqz)
        m12 = 2 * (qy*qz - qw*qx)
        m20 = 2 * (qx*qz - qw*qy)
        m21 = 2 * (qy*qz + qw*qx)
        m22 = 1 - 2 * (sqx + sqy)

        self.rotation_matrix = np.array([
            [m00, m01, m02, translation[0]],
            [m10, m11, m12, translation[1]],
            [m20, m21, m22, translation[2]],
            [0, 0, 0, 1]
        ])

        self.odom_data = msg  # 保存该消息

    def pointcloud_callback(self, msg):
        if self.odom_data is None or self.rotation_matrix is None:
            return

        # 将点云读取为 NumPy 数组（N x 3）
        points_list = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([p[0], p[1], p[2]])
        points = np.array(points_list, dtype=np.float32)

        # 添加齐次坐标列（变为 N x 4）
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))

        # 应用 4x4 位姿变换矩阵（旋转+平移）
        transformed_points = np.dot(self.rotation_matrix, points_homogeneous.T).T[:, :3]

        # 构造新的 PointCloud2 消息，设置目标 frame_id
        header = msg.header
        header.frame_id = self.get_parameter('frame_id').value
        transformed_msg = pc2.create_cloud_xyz32(header, transformed_points)

        self.transformed_pointcloud_pub.publish(transformed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()