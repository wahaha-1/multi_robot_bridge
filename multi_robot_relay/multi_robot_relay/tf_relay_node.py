#!/usr/bin/env python3
# encoding: utf-8
"""
TF变换中转节点 - TF Relay Node

功能：监听本地TF变换，添加机器人命名空间前缀后重新广播
- 保持本地TF框架不变（用于本地导航）
- 同时广播带命名空间的TF（用于多机器人可视化）
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros
from geometry_msgs.msg import TransformStamped


class TFRelayNode(Node):
    """TF变换中转节点"""
    
    def __init__(self):
        super().__init__('tf_relay_node')
        
        # 声明参数
        self.declare_parameter('robot_name', '')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('enabled', True)
        
        # 获取参数
        self.robot_name = self.get_parameter('robot_name').value
        if not self.robot_name:
            self.robot_name = os.environ.get('ROBOT_NAME', os.environ.get('HOST', 'robot1'))
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enabled = self.get_parameter('enabled').value
        
        if not self.enabled:
            self.get_logger().info('TF中转已禁用')
            return
        
        self.get_logger().info(f'机器人名称: {self.robot_name}')
        
        # 需要中转的TF框架
        self.frames_to_relay = [
            'map',
            'odom',
            'base_footprint',
            'base_link',
            'laser',
            'camera_link',
            'imu_link',
        ]
        
        # TF监听器和广播器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # 创建定时器
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.relay_tf_callback)
        
        # 静态变换已发布标记
        self.static_transforms_published = set()
        
        self.get_logger().info('\033[1;32mTF中转节点启动成功\033[0m')
    
    def relay_tf_callback(self):
        """定时器回调：中转TF变换"""
        if not self.enabled:
            return
        
        current_time = self.get_clock().now()
        
        # 遍历需要中转的框架
        for i in range(len(self.frames_to_relay)):
            for j in range(len(self.frames_to_relay)):
                if i == j:
                    continue
                
                source_frame = self.frames_to_relay[i]
                target_frame = self.frames_to_relay[j]
                
                try:
                    # 查询TF变换
                    trans = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        Time(),
                        timeout=Duration(seconds=0.01)
                    )
                    
                    # 创建新的变换，添加机器人名称前缀
                    new_trans = TransformStamped()
                    new_trans.header.stamp = current_time.to_msg()
                    new_trans.header.frame_id = f'{self.robot_name}/{target_frame}'
                    new_trans.child_frame_id = f'{self.robot_name}/{source_frame}'
                    new_trans.transform = trans.transform
                    
                    # 广播变换
                    self.tf_broadcaster.sendTransform(new_trans)
                    
                except (tf2_ros.LookupException, 
                       tf2_ros.ConnectivityException, 
                       tf2_ros.ExtrapolationException) as e:
                    # TF不可用，跳过
                    pass
    
    def publish_static_transform(self, parent_frame, child_frame, x=0.0, y=0.0, z=0.0, 
                                 qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """发布静态变换"""
        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = f'{self.robot_name}/{parent_frame}'
        trans.child_frame_id = f'{self.robot_name}/{child_frame}'
        trans.transform.translation.x = x
        trans.transform.translation.y = y
        trans.transform.translation.z = z
        trans.transform.rotation.x = qx
        trans.transform.rotation.y = qy
        trans.transform.rotation.z = qz
        trans.transform.rotation.w = qw
        
        self.static_tf_broadcaster.sendTransform(trans)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = TFRelayNode()
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
