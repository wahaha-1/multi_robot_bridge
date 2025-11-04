#!/usr/bin/env python3
# encoding: utf-8
"""
单个机器人中转站启动文件

在每个机器人上运行，启动话题中转、TF中转和坐标转换节点
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
    
    # 声明启动参数
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='机器人名称 (如: robot1, robot2)'
    )
    
    enable_tf_relay_arg = DeclareLaunchArgument(
        'enable_tf_relay',
        default_value='true',
        description='是否启用TF中转'
    )
    
    enable_coord_transform_arg = DeclareLaunchArgument(
        'enable_coord_transform',
        default_value='false',
        description='是否启用坐标转换'
    )
    
    map_offset_x_arg = DeclareLaunchArgument(
        'map_offset_x',
        default_value='0.0',
        description='地图X轴偏移 (米)'
    )
    
    map_offset_y_arg = DeclareLaunchArgument(
        'map_offset_y',
        default_value='0.0',
        description='地图Y轴偏移 (米)'
    )
    
    map_offset_yaw_arg = DeclareLaunchArgument(
        'map_offset_yaw',
        default_value='0.0',
        description='地图旋转偏移 (弧度)'
    )
    
    # 话题中转节点
    topic_relay_node = Node(
        package='multi_robot_relay',
        executable='topic_relay',
        name='topic_relay_node',
        output='screen',
        parameters=[
            config_file,
            {'robot_name': LaunchConfiguration('robot_name')}
        ]
    )
    
    # TF中转节点
    tf_relay_node = Node(
        package='multi_robot_relay',
        executable='tf_relay',
        name='tf_relay_node',
        output='screen',
        parameters=[
            config_file,
            {'robot_name': LaunchConfiguration('robot_name')}
        ]
    )
    
    # 坐标转换节点
    coord_transform_node = Node(
        package='multi_robot_relay',
        executable='coordinate_transformer',
        name='coordinate_transformer_node',
        output='screen',
        parameters=[
            config_file,
            {
                'robot_name': LaunchConfiguration('robot_name'),
                'map_offset_x': LaunchConfiguration('map_offset_x'),
                'map_offset_y': LaunchConfiguration('map_offset_y'),
                'map_offset_yaw': LaunchConfiguration('map_offset_yaw'),
            }
        ]
    )
    
    return LaunchDescription([
        robot_name_arg,
        enable_tf_relay_arg,
        enable_coord_transform_arg,
        map_offset_x_arg,
        map_offset_y_arg,
        map_offset_yaw_arg,
        topic_relay_node,
        # tf_relay_node,  # 需要时取消注释
        # coord_transform_node,  # 需要时取消注释
    ])


if __name__ == '__main__':
    generate_launch_description()
