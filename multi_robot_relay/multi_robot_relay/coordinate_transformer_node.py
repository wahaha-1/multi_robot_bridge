#!/usr/bin/env python3
# encoding: utf-8
"""
坐标转换器节点 - Coordinate Transformer Node

功能：处理多机器人坐标系统不统一的问题
- 如果所有机器人使用同一地图原点，此节点可禁用
- 如果机器人地图原点不同，通过配置偏移量实现坐标统一
- 支持对位置、速度等进行坐标变换
"""

import os
import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class CoordinateTransformerNode(Node):
    """坐标转换器节点"""
    
    def __init__(self):
        super().__init__('coordinate_transformer_node')
        
        # 声明参数
        self.declare_parameter('robot_name', '')
        self.declare_parameter('enabled', False)
        self.declare_parameter('map_offset_x', 0.0)
        self.declare_parameter('map_offset_y', 0.0)
        self.declare_parameter('map_offset_yaw', 0.0)
        
        # 获取参数
        self.robot_name = self.get_parameter('robot_name').value
        if not self.robot_name:
            self.robot_name = os.environ.get('ROBOT_NAME', os.environ.get('HOST', 'robot1'))
        
        self.enabled = self.get_parameter('enabled').value
        
        if not self.enabled:
            self.get_logger().info('坐标转换器已禁用 - 假设所有机器人使用同一地图原点')
            return
        
        # 地图原点偏移量
        self.offset_x = self.get_parameter('map_offset_x').value
        self.offset_y = self.get_parameter('map_offset_y').value
        self.offset_yaw = self.get_parameter('map_offset_yaw').value
        
        self.get_logger().info(f'机器人名称: {self.robot_name}')
        self.get_logger().info(f'地图偏移: x={self.offset_x}, y={self.offset_y}, yaw={self.offset_yaw}')
        
        # 计算旋转矩阵
        self.cos_yaw = math.cos(self.offset_yaw)
        self.sin_yaw = math.sin(self.offset_yaw)
        
        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # 发布map之间的变换
        self.publish_map_transform()
        
        # 创建订阅和发布
        self.create_subscribers_publishers()
        
        self.get_logger().info('\033[1;32m坐标转换器节点启动成功\033[0m')
    
    def publish_map_transform(self):
        """发布本地map到全局map的静态变换"""
        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = 'global_map'
        trans.child_frame_id = f'{self.robot_name}/map'
        
        trans.transform.translation.x = self.offset_x
        trans.transform.translation.y = self.offset_y
        trans.transform.translation.z = 0.0
        
        # 将yaw角转换为四元数
        q = quaternion_from_euler(0, 0, self.offset_yaw)
        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]
        
        self.static_tf_broadcaster.sendTransform(trans)
        self.get_logger().info('已发布地图变换: global_map → {}/map'.format(self.robot_name))
    
    def create_subscribers_publishers(self):
        """创建订阅者和发布者"""
        # 里程计坐标转换
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            f'/{self.robot_name}/odom_global',
            10
        )
        
        # AMCL位姿坐标转换
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{self.robot_name}/amcl_pose',
            self.amcl_callback,
            10
        )
        self.amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            f'/{self.robot_name}/amcl_pose_global',
            10
        )
        
        # 目标点坐标转换 (全局 → 本地)
        self.goal_global_sub = self.create_subscription(
            PoseStamped,
            f'/{self.robot_name}/goal_pose_global',
            self.goal_global_callback,
            10
        )
        self.goal_local_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/goal_pose',
            10
        )
    
    def transform_pose_local_to_global(self, x, y, yaw):
        """
        将本地坐标转换为全局坐标
        
        Args:
            x, y, yaw: 本地坐标系中的位姿
            
        Returns:
            x_global, y_global, yaw_global: 全局坐标系中的位姿
        """
        # 旋转
        x_rotated = x * self.cos_yaw - y * self.sin_yaw
        y_rotated = x * self.sin_yaw + y * self.cos_yaw
        
        # 平移
        x_global = x_rotated + self.offset_x
        y_global = y_rotated + self.offset_y
        yaw_global = yaw + self.offset_yaw
        
        # 归一化角度到 [-pi, pi]
        while yaw_global > math.pi:
            yaw_global -= 2 * math.pi
        while yaw_global < -math.pi:
            yaw_global += 2 * math.pi
        
        return x_global, y_global, yaw_global
    
    def transform_pose_global_to_local(self, x, y, yaw):
        """
        将全局坐标转换为本地坐标
        
        Args:
            x, y, yaw: 全局坐标系中的位姿
            
        Returns:
            x_local, y_local, yaw_local: 本地坐标系中的位姿
        """
        # 反向平移
        x_translated = x - self.offset_x
        y_translated = y - self.offset_y
        
        # 反向旋转
        x_local = x_translated * self.cos_yaw + y_translated * self.sin_yaw
        y_local = -x_translated * self.sin_yaw + y_translated * self.cos_yaw
        yaw_local = yaw - self.offset_yaw
        
        # 归一化角度到 [-pi, pi]
        while yaw_local > math.pi:
            yaw_local -= 2 * math.pi
        while yaw_local < -math.pi:
            yaw_local += 2 * math.pi
        
        return x_local, y_local, yaw_local
    
    def odom_callback(self, msg):
        """里程计回调：转换为全局坐标"""
        # 提取位姿
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # 转换为全局坐标
        x_global, y_global, yaw_global = self.transform_pose_local_to_global(x, y, yaw)
        
        # 创建新消息
        global_msg = Odometry()
        global_msg.header = msg.header
        global_msg.header.frame_id = 'global_map'
        global_msg.child_frame_id = f'{self.robot_name}/base_footprint'
        
        global_msg.pose.pose.position.x = x_global
        global_msg.pose.pose.position.y = y_global
        global_msg.pose.pose.position.z = msg.pose.pose.position.z
        
        q_global = quaternion_from_euler(0, 0, yaw_global)
        global_msg.pose.pose.orientation.x = q_global[0]
        global_msg.pose.pose.orientation.y = q_global[1]
        global_msg.pose.pose.orientation.z = q_global[2]
        global_msg.pose.pose.orientation.w = q_global[3]
        
        # 速度保持不变（假设在机器人坐标系中）
        global_msg.twist = msg.twist
        
        # 发布
        self.odom_pub.publish(global_msg)
    
    def amcl_callback(self, msg):
        """AMCL位姿回调：转换为全局坐标"""
        # 提取位姿
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # 转换为全局坐标
        x_global, y_global, yaw_global = self.transform_pose_local_to_global(x, y, yaw)
        
        # 创建新消息
        global_msg = PoseWithCovarianceStamped()
        global_msg.header = msg.header
        global_msg.header.frame_id = 'global_map'
        
        global_msg.pose.pose.position.x = x_global
        global_msg.pose.pose.position.y = y_global
        global_msg.pose.pose.position.z = msg.pose.pose.position.z
        
        q_global = quaternion_from_euler(0, 0, yaw_global)
        global_msg.pose.pose.orientation.x = q_global[0]
        global_msg.pose.pose.orientation.y = q_global[1]
        global_msg.pose.pose.orientation.z = q_global[2]
        global_msg.pose.pose.orientation.w = q_global[3]
        
        global_msg.pose.covariance = msg.pose.covariance
        
        # 发布
        self.amcl_pub.publish(global_msg)
    
    def goal_global_callback(self, msg):
        """全局目标点回调：转换为本地坐标"""
        # 提取位姿
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # 转换为本地坐标
        x_local, y_local, yaw_local = self.transform_pose_global_to_local(x, y, yaw)
        
        # 创建新消息
        local_msg = PoseStamped()
        local_msg.header = msg.header
        local_msg.header.frame_id = 'map'
        
        local_msg.pose.position.x = x_local
        local_msg.pose.position.y = y_local
        local_msg.pose.position.z = msg.pose.position.z
        
        q_local = quaternion_from_euler(0, 0, yaw_local)
        local_msg.pose.orientation.x = q_local[0]
        local_msg.pose.orientation.y = q_local[1]
        local_msg.pose.orientation.z = q_local[2]
        local_msg.pose.orientation.w = q_local[3]
        
        # 发布
        self.goal_local_pub.publish(local_msg)
        self.get_logger().info(f'目标点转换: 全局({x:.2f}, {y:.2f}) → 本地({x_local:.2f}, {y_local:.2f})')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = CoordinateTransformerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
