#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 节点：将 /odom 信息转换为 map → odom 的 TF 变换
用途：为不使用 AMCL 的系统提供基本 tf 坐标转换支持
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomToMapTransformer(Node):
    def __init__(self):
        super().__init__('odom_to_map_transformer')

        # 初始化 TF 广播器：用于发布 map → odom 动态变换
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 可选：静态 TF 广播器，未启用（见 publish_static_transform）
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # 订阅 odometry 数据（通常由机器人底盘发布）
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 若需发布 odom → base_link 静态 TF，可启用以下函数
        # self.publish_static_transform()

    def odom_callback(self, odom_msg):
        # 动态发布 map → odom 坐标变换，假设 odom 即为机器人的实际位姿
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def publish_static_transform(self):
        # （未启用）可发布 odom → base_link 的静态变换
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'odom'
        static_transform.child_frame_id = 'base_link'

        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0  # 无旋转

        # self.static_tf_broadcaster.sendTransform(static_transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToMapTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()