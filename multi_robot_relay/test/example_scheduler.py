#!/usr/bin/env python3
# encoding: utf-8
"""
示例：多机器人任务调度器

从中央控制台发送导航目标给多个机器人
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math


class MultiRobotScheduler(Node):
    """多机器人任务调度器"""
    
    def __init__(self):
        super().__init__('multi_robot_scheduler')
        
        # 机器人列表
        self.robot_names = ['robot1', 'robot2', 'robot3']
        
        # 机器人当前位置
        self.robot_positions = {}
        
        # 为每个机器人创建发布者和订阅者
        self.goal_publishers = {}
        self.pose_subscribers = {}
        
        for robot_name in self.robot_names:
            # 目标点发布者
            self.goal_publishers[robot_name] = self.create_publisher(
                PoseStamped,
                f'/{robot_name}/goal_pose',
                10
            )
            
            # 位置订阅者
            self.pose_subscribers[robot_name] = self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{robot_name}/amcl_pose',
                lambda msg, name=robot_name: self.pose_callback(msg, name),
                10
            )
            
            self.robot_positions[robot_name] = None
        
        self.get_logger().info('多机器人任务调度器启动')
        self.get_logger().info(f'管理机器人: {", ".join(self.robot_names)}')
    
    def pose_callback(self, msg, robot_name):
        """位置回调"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_positions[robot_name] = (x, y)
        # self.get_logger().info(f'{robot_name} 位置: ({x:.2f}, {y:.2f})')
    
    def send_goal(self, robot_name, x, y, yaw=0.0):
        """
        发送导航目标给指定机器人
        
        Args:
            robot_name: 机器人名称
            x, y: 目标位置 (米)
            yaw: 目标朝向 (弧度)
        """
        if robot_name not in self.goal_publishers:
            self.get_logger().error(f'未知的机器人: {robot_name}')
            return False
        
        # 创建目标消息
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        
        # 将yaw转换为四元数
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 发布目标
        self.goal_publishers[robot_name].publish(goal_msg)
        self.get_logger().info(f'发送目标给 {robot_name}: ({x:.2f}, {y:.2f}, {yaw:.2f})')
        
        return True
    
    def send_goals_batch(self, goals):
        """
        批量发送目标
        
        Args:
            goals: 列表，每个元素为 (robot_name, x, y, yaw)
        """
        for goal in goals:
            robot_name, x, y, yaw = goal
            self.send_goal(robot_name, x, y, yaw)
    
    def get_robot_position(self, robot_name):
        """获取机器人当前位置"""
        return self.robot_positions.get(robot_name)
    
    def print_all_positions(self):
        """打印所有机器人位置"""
        self.get_logger().info('=== 机器人位置 ===')
        for robot_name in self.robot_names:
            pos = self.robot_positions.get(robot_name)
            if pos:
                x, y = pos
                self.get_logger().info(f'  {robot_name}: ({x:.2f}, {y:.2f})')
            else:
                self.get_logger().info(f'  {robot_name}: 未知')


def main():
    """主函数"""
    rclpy.init()
    
    scheduler = MultiRobotScheduler()
    
    try:
        # 等待一下，让订阅者连接
        import time
        time.sleep(2.0)
        
        # 示例1：给每个机器人发送不同的目标
        scheduler.get_logger().info('\n=== 示例1：分配不同目标 ===')
        scheduler.send_goal('robot1', 1.0, 0.0, 0.0)
        time.sleep(0.5)
        scheduler.send_goal('robot2', 0.0, 1.0, 1.57)  # 90度
        time.sleep(0.5)
        scheduler.send_goal('robot3', -1.0, 0.0, 3.14)  # 180度
        
        time.sleep(2.0)
        
        # 示例2：批量发送目标
        scheduler.get_logger().info('\n=== 示例2：批量分配目标 ===')
        goals = [
            ('robot1', 2.0, 2.0, 0.0),
            ('robot2', 2.0, -2.0, 0.0),
            ('robot3', -2.0, 2.0, 0.0),
        ]
        scheduler.send_goals_batch(goals)
        
        # 持续运行，监控位置
        scheduler.get_logger().info('\n=== 监控机器人位置（按Ctrl+C退出） ===')
        while rclpy.ok():
            rclpy.spin_once(scheduler, timeout_sec=1.0)
            scheduler.print_all_positions()
            time.sleep(5.0)
    
    except KeyboardInterrupt:
        scheduler.get_logger().info('任务调度器停止')
    finally:
        scheduler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
