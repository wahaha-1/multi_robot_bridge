#!/usr/bin/env python3
# encoding: utf-8
"""
测试脚本：验证话题中转功能

测试本地话题是否正确中转到全局话题
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time


class RelayTester(Node):
    """中转测试节点"""
    
    def __init__(self):
        super().__init__('relay_tester')
        
        self.robot_name = 'robot1'
        self.received_local = False
        self.received_global = False
        
        # 发布到本地话题
        self.local_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        
        # 订阅全局话题（应该从中转站收到）
        self.global_sub = self.create_subscription(
            Twist,
            f'/{self.robot_name}/cmd_vel_status',
            self.global_callback,
            10
        )
        
        # 创建定时器发布测试消息
        self.timer = self.create_timer(1.0, self.publish_test_msg)
        
        self.get_logger().info('中转测试节点启动')
    
    def publish_test_msg(self):
        """发布测试消息到本地话题"""
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.1
        
        self.local_pub.publish(msg)
        self.get_logger().info('发布测试消息到本地话题: /controller/cmd_vel')
    
    def global_callback(self, msg):
        """全局话题回调"""
        self.received_global = True
        self.get_logger().info(f'✓ 从全局话题收到消息: /{self.robot_name}/cmd_vel_status')
        self.get_logger().info(f'  linear.x={msg.linear.x}, angular.z={msg.angular.z}')


def main():
    """主函数"""
    rclpy.init()
    
    node = RelayTester()
    
    try:
        # 运行5秒
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # 检查结果
        if node.received_global:
            node.get_logger().info('\033[1;32m✓ 测试通过：话题中转正常工作\033[0m')
        else:
            node.get_logger().error('\033[1;31m✗ 测试失败：未收到全局话题消息\033[0m')
            node.get_logger().error('  请检查中转节点是否运行：')
            node.get_logger().error('  ros2 launch multi_robot_relay robot_relay.launch.py robot_name:=robot1')
    
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
