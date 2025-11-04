#!/usr/bin/env python3
# encoding: utf-8
"""
中央注册服务器启动文件

在中央服务器或其中一台机器人上运行，用于管理所有机器人
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    pkg_path = get_package_share_directory('multi_robot_relay')
    config_file = os.path.join(pkg_path, 'config', 'relay_config.yaml')
    
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'robot_timeout',
            default_value='10.0',
            description='机器人离线超时时间 (秒)'
        ),
        
        # 机器人注册节点
        Node(
            package='multi_robot_relay',
            executable='robot_registry',
            name='robot_registry_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'update_rate': 1.0,
                    'robot_timeout': LaunchConfiguration('robot_timeout'),
                    'auto_broadcast': True,
                }
            ]
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
