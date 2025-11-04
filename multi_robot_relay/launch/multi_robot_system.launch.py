#!/usr/bin/env python3
# encoding: utf-8
"""
多机器人系统完整启动文件

同时启动多个机器人的中转站（用于测试或集中式部署）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    pkg_path = get_package_share_directory('multi_robot_relay')
    robot_relay_launch = os.path.join(pkg_path, 'launch', 'robot_relay.launch.py')
    central_registry_launch = os.path.join(pkg_path, 'launch', 'central_registry.launch.py')
    
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'num_robots',
            default_value='2',
            description='机器人数量'
        ),
        
        # 启动中央注册服务器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(central_registry_launch)
        ),
        
        # 启动机器人1的中转站
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_relay_launch),
            launch_arguments={
                'robot_name': 'robot1',
                'enable_coord_transform': 'false',
            }.items()
        ),
        
        # 启动机器人2的中转站
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_relay_launch),
            launch_arguments={
                'robot_name': 'robot2',
                'enable_coord_transform': 'false',
            }.items()
        ),
        
        # 如果需要更多机器人，继续添加...
    ])


if __name__ == '__main__':
    generate_launch_description()
