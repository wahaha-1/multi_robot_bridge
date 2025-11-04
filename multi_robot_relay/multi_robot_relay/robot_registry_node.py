#!/usr/bin/env python3
# encoding: utf-8
"""
机器人注册节点 - Robot Registry Node

功能：中央注册服务器，管理所有在线机器人
- 接收机器人注册和心跳
- 维护机器人列表
- 广播机器人状态
- 提供查询服务
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import time


class RobotInfo:
    """机器人信息类"""
    
    def __init__(self, name, ip='', status='online'):
        self.name = name
        self.ip = ip
        self.status = status
        self.last_heartbeat = time.time()
        self.position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    
    def update_heartbeat(self):
        """更新心跳时间"""
        self.last_heartbeat = time.time()
        self.status = 'online'
    
    def is_timeout(self, timeout=10.0):
        """检查是否超时"""
        return (time.time() - self.last_heartbeat) > timeout
    
    def to_dict(self):
        """转换为字典"""
        return {
            'name': self.name,
            'ip': self.ip,
            'status': self.status,
            'last_heartbeat': self.last_heartbeat,
            'position': self.position
        }


class RobotRegistryNode(Node):
    """机器人注册节点"""
    
    def __init__(self):
        super().__init__('robot_registry_node')
        
        # 声明参数
        self.declare_parameter('update_rate', 1.0)
        self.declare_parameter('robot_timeout', 10.0)
        self.declare_parameter('auto_broadcast', True)
        
        # 获取参数
        self.update_rate = self.get_parameter('update_rate').value
        self.robot_timeout = self.get_parameter('robot_timeout').value
        self.auto_broadcast = self.get_parameter('auto_broadcast').value
        
        # 机器人列表
        self.robots = {}  # {robot_name: RobotInfo}
        
        # 创建服务
        self.register_srv = self.create_service(
            Trigger,
            '/multi_robot/register',
            self.register_callback
        )
        
        self.get_robots_srv = self.create_service(
            Trigger,
            '/multi_robot/get_robots',
            self.get_robots_callback
        )
        
        # 创建发布者
        self.robot_list_pub = self.create_publisher(
            String,
            '/multi_robot/robot_list',
            10
        )
        
        # 创建定时器
        if self.auto_broadcast:
            timer_period = 1.0 / self.update_rate
            self.timer = self.create_timer(timer_period, self.update_callback)
        
        self.get_logger().info('\033[1;32m机器人注册节点启动成功\033[0m')
        self.get_logger().info(f'机器人超时时间: {self.robot_timeout}秒')
    
    def register_callback(self, request, response):
        """注册服务回调"""
        # 从请求中获取机器人名称（通过话题命名空间）
        # 这里简化处理，实际应该从消息中获取
        robot_name = 'unknown'
        
        if robot_name not in self.robots:
            self.robots[robot_name] = RobotInfo(robot_name)
            self.get_logger().info(f'新机器人注册: {robot_name}')
        else:
            self.robots[robot_name].update_heartbeat()
        
        response.success = True
        response.message = f'Robot {robot_name} registered'
        return response
    
    def get_robots_callback(self, request, response):
        """获取机器人列表服务回调"""
        robot_list = [info.to_dict() for info in self.robots.values()]
        response.success = True
        response.message = json.dumps(robot_list)
        return response
    
    def update_callback(self):
        """定时更新回调"""
        # 检查机器人超时
        timeout_robots = []
        for name, info in self.robots.items():
            if info.is_timeout(self.robot_timeout):
                info.status = 'offline'
                timeout_robots.append(name)
        
        if timeout_robots:
            self.get_logger().warn(f'机器人超时: {", ".join(timeout_robots)}')
        
        # 广播机器人列表
        if self.auto_broadcast:
            robot_list = [info.to_dict() for info in self.robots.values()]
            msg = String()
            msg.data = json.dumps(robot_list)
            self.robot_list_pub.publish(msg)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = RobotRegistryNode()
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
