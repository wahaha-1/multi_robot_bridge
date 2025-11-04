#!/usr/bin/env python3
# encoding: utf-8
"""
话题中转节点 - Topic Relay Node

功能：在本地话题和全局话题之间进行双向中转
- 上行: 本地话题 → 全局话题 (添加机器人命名空间)
- 下行: 全局话题 → 本地话题 (移除命名空间)

不修改原有节点代码，通过中转层实现多机器人话题隔离
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import importlib
from threading import Lock
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class TopicRelayNode(Node):
    """话题中转节点"""
    
    def __init__(self):
        super().__init__('topic_relay_node')
        
        # 声明并获取参数
        self.declare_parameter('robot_name', '')
        self.robot_name = self.get_parameter('robot_name').value
        if not self.robot_name:
            self.robot_name = os.environ.get('ROBOT_NAME', os.environ.get('HOST', 'robot1'))
        
        self.get_logger().info(f'机器人名称: {self.robot_name}')

        # 声明参数并指定类型（字符串数组）
        string_array_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING_ARRAY,
            description='String array parameter'
        )
        integer_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Integer parameter'
        )
        
        # 先声明参数，不设置默认值
        self.declare_parameter('uplink_local_topics', descriptor=string_array_descriptor)
        self.declare_parameter('uplink_global_topics', descriptor=string_array_descriptor)
        self.declare_parameter('uplink_msg_types', descriptor=string_array_descriptor)
        self.declare_parameter('uplink_qos_depth', descriptor=integer_descriptor)
        
        self.declare_parameter('downlink_global_topics', descriptor=string_array_descriptor)
        self.declare_parameter('downlink_local_topics', descriptor=string_array_descriptor)
        self.declare_parameter('downlink_msg_types', descriptor=string_array_descriptor)
        self.declare_parameter('downlink_qos_depth', descriptor=integer_descriptor)
        
        # 获取参数值，如果参数未设置则使用默认值
        self.uplink_local_topics = self.get_parameter('uplink_local_topics').value or []
        self.uplink_global_topics = self.get_parameter('uplink_global_topics').value or []
        self.uplink_msg_types = self.get_parameter('uplink_msg_types').value or []
        self.uplink_qos_depth = self.get_parameter('uplink_qos_depth').value or 10

        self.downlink_global_topics = self.get_parameter('downlink_global_topics').value or []
        self.downlink_local_topics = self.get_parameter('downlink_local_topics').value or []
        self.downlink_msg_types = self.get_parameter('downlink_msg_types').value or []
        self.downlink_qos_depth = self.get_parameter('downlink_qos_depth').value or 10

        
        # 存储订阅者和发布者
        self.uplink_subs = []    # 上行订阅者 (本地话题)
        self.uplink_pubs = []    # 上行发布者 (全局话题)
        self.downlink_subs = []  # 下行订阅者 (全局话题)
        self.downlink_pubs = []  # 下行发布者 (本地话题)
        
        # 消息类型缓存
        self.msg_type_cache = {}
        
        # 线程锁
        self.lock = Lock()
        
        # 创建中转器
        self.create_relays()
        
        self.get_logger().info('\033[1;32m话题中转节点启动成功\033[0m')
    
    def get_msg_class(self, msg_type_str):
        """
        根据消息类型字符串获取消息类
        支持格式: 'nav_msgs/Odometry' 或 'nav_msgs/msg/Odometry'
        """
        if msg_type_str in self.msg_type_cache:
            return self.msg_type_cache[msg_type_str]
        
        try:
            # 分割包名和消息名
            parts = msg_type_str.split('/')
            
            if len(parts) == 3:
                # 格式: 'nav_msgs/msg/Odometry'
                package_name = parts[0]
                msg_name = parts[2]
            elif len(parts) == 2:
                # 格式: 'nav_msgs/Odometry'
                package_name = parts[0]
                msg_name = parts[1]
            else:
                self.get_logger().error(f'无效的消息类型格式: {msg_type_str}')
                return None
            
            # 动态导入消息类
            module = importlib.import_module(f'{package_name}.msg')
            msg_class = getattr(module, msg_name)
            
            # 缓存消息类
            self.msg_type_cache[msg_type_str] = msg_class
            return msg_class
            
        except Exception as e:
            self.get_logger().error(f'无法加载消息类型 {msg_type_str}: {e}')
            return None

    def create_qos(self, depth=10):
        """创建QoS配置"""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=int(depth) if depth is not None else 10,
        )
    
    
    
    def create_relays(self):
        """创建所有中转器"""
        # 创建上行中转器 (本地 → 全局)
        self.create_uplink_relays()
        
        # 创建下行中转器 (全局 → 本地)
        self.create_downlink_relays()
    
    def create_uplink_relays(self):
        """创建上行中转器 (本地话题 → 全局话题)"""
        local_topics = self.uplink_local_topics
        global_topics = self.uplink_global_topics
        msg_types = self.uplink_msg_types
        qos_depth = self.uplink_qos_depth
        
        if not local_topics:
            self.get_logger().warn('未配置上行话题')
            return
        
        if len(local_topics) != len(global_topics) or len(local_topics) != len(msg_types):
            self.get_logger().error('上行话题配置长度不匹配')
            return
        
        for i, local_topic in enumerate(local_topics):
            global_topic_template = global_topics[i]
            msg_type_str = msg_types[i]
            
            # 替换机器人名称
            global_topic = global_topic_template.format(robot_name=self.robot_name)
            
            # 获取消息类
            msg_class = self.get_msg_class(msg_type_str)
            if msg_class is None:
                continue
            
            try:
                # 创建发布者
                qos = self.create_qos(qos_depth)
                pub = self.create_publisher(msg_class, global_topic, qos)
                self.uplink_pubs.append(pub)
                
                # 创建订阅者，使用lambda闭包捕获pub
                def callback(msg, publisher=pub):
                    with self.lock:
                        publisher.publish(msg)
                
                sub = self.create_subscription(msg_class, local_topic, callback, qos)
                self.uplink_subs.append(sub)
                
                self.get_logger().info(f'上行中转: {local_topic} → {global_topic}')
            except Exception as e:
                self.get_logger().error(f'创建上行中转失败 {local_topic}: {e}')
    
    def create_downlink_relays(self):
        """创建下行中转器 (全局话题 → 本地话题)"""
        global_topics = self.downlink_global_topics
        local_topics = self.downlink_local_topics
        msg_types = self.downlink_msg_types
        qos_depth = self.downlink_qos_depth
        
        if not global_topics:
            self.get_logger().warn('未配置下行话题')
            return
        
        if len(global_topics) != len(local_topics) or len(global_topics) != len(msg_types):
            self.get_logger().error('下行话题配置长度不匹配')
            return
        
        for i, global_topic_template in enumerate(global_topics):
            local_topic = local_topics[i]
            msg_type_str = msg_types[i]
            
            # 替换机器人名称
            global_topic = global_topic_template.format(robot_name=self.robot_name)
            
            # 获取消息类
            msg_class = self.get_msg_class(msg_type_str)
            if msg_class is None:
                continue
            
            try:
                # 创建发布者
                qos = self.create_qos(qos_depth)
                pub = self.create_publisher(msg_class, local_topic, qos)
                self.downlink_pubs.append(pub)
                
                # 创建订阅者，使用lambda闭包捕获pub
                def callback(msg, publisher=pub):
                    with self.lock:
                        publisher.publish(msg)
                
                sub = self.create_subscription(msg_class, global_topic, callback, qos)
                self.downlink_subs.append(sub)
                
                self.get_logger().info(f'下行中转: {global_topic} → {local_topic}')
            except Exception as e:
                self.get_logger().error(f'创建下行中转失败 {global_topic}: {e}')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = TopicRelayNode()
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
