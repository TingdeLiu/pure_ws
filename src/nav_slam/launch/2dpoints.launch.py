#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航 SLAM 启动文件（ROS 2）
本文件以 LaunchDescription 形式启动各个功能节点。
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 LaunchDescription 对象。"""
    ld = LaunchDescription()

    # ---------------------------- 配置路径 ---------------------------- #
    package_name = 'nav_slam'                                             # 包名
    config_dir = get_package_share_directory(package_name)               # share 目录
    rviz_config_file = os.path.join(config_dir, 'config', 'rviz.rviz')   # rviz 配置

    # ---------------------------- 节点定义 ---------------------------- #
    rrt = Node(                             # 基于 RRT 的路径规划
        package='nav_slam',
        executable='rrt',
        name='rrt',
        output='screen',
    )

    map_pub = Node(                         # 发布 2D 地图
        package='nav_slam',
        executable='map_pub',
        name='map_pub',
        output='screen',
    )

    odom_map_tf = Node(                     # 发布 odom→map 变换
        package='nav_slam',
        executable='odom_map_tf',
        name='odom_map_tf',
        output='screen',
    )

    points_pub_map = Node(                  # 发布优化点云
        package='nav_slam',
        executable='points_pub_map',
        name='points_pub_map',
        output='screen',
    )

    start_nav_dwa = Node(                   # DWA 运动控制
        package='nav_slam',
        executable='start_nav_dwa',
        name='start_nav_dwa',
        output='screen',
    )

    rviz2_node = Node(                      # RViz2 可视化
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # ---------------------------- 添加到 LaunchDescription ---------------------------- #
    for action in (
        rrt,
        map_pub,
        odom_map_tf,
        points_pub_map,
        start_nav_dwa,
        rviz2_node,
    ):
        ld.add_action(action)

    return ld
