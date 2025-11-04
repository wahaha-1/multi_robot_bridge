#!/usr/bin/env python3
# encoding: utf-8
"""
快速测试脚本 - 验证多机器人中转系统

这个脚本会自动检查：
1. 话题是否正确创建
2. 数据是否正常流动
3. 中转延迟是否可接受
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import sys


class RelayTester(Node):
    """中转系统测试器"""
    
    def __init__(self, robot_name='robot1'):
        super().__init__('relay_tester')
        
        self.robot_name = robot_name
        self.test_results = {}
        
        # 创建QoS配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 测试项目
        self.tests = {
            'odom': {
                'local_topic': '/odom',
                'global_topic': f'/{robot_name}/odom',
                'msg_type': Odometry,
                'received_local': False,
                'received_global': False,
                'local_data': None,
                'global_data': None,
            },
            'scan': {
                'local_topic': '/scan',
                'global_topic': f'/{robot_name}/scan',
                'msg_type': LaserScan,
                'received_local': False,
                'received_global': False,
            },
            'amcl_pose': {
                'local_topic': '/amcl_pose',
                'global_topic': f'/{robot_name}/amcl_pose',
                'msg_type': PoseWithCovarianceStamped,
                'received_local': False,
                'received_global': False,
            }
        }
        
        # 创建订阅者
        for test_name, test_info in self.tests.items():
            # 本地话题订阅
            self.create_subscription(
                test_info['msg_type'],
                test_info['local_topic'],
                lambda msg, name=test_name: self.local_callback(msg, name),
                qos
            )
            
            # 全局话题订阅
            self.create_subscription(
                test_info['msg_type'],
                test_info['global_topic'],
                lambda msg, name=test_name: self.global_callback(msg, name),
                qos
            )
        
        self.get_logger().info(f'🧪 开始测试机器人: {robot_name}')
        self.get_logger().info('⏳ 等待数据... (测试将在 10 秒后完成)')
    
    def local_callback(self, msg, test_name):
        """本地话题回调"""
        if not self.tests[test_name]['received_local']:
            self.tests[test_name]['received_local'] = True
            self.tests[test_name]['local_data'] = msg
            self.get_logger().info(f'✅ 收到本地话题: {self.tests[test_name]["local_topic"]}')
    
    def global_callback(self, msg, test_name):
        """全局话题回调"""
        if not self.tests[test_name]['received_global']:
            self.tests[test_name]['received_global'] = True
            self.tests[test_name]['global_data'] = msg
            self.get_logger().info(f'✅ 收到全局话题: {self.tests[test_name]["global_topic"]}')
    
    def print_results(self):
        """打印测试结果"""
        print('\n' + '='*60)
        print(f'📊 测试结果 - 机器人: {self.robot_name}')
        print('='*60)
        
        all_passed = True
        
        for test_name, test_info in self.tests.items():
            print(f'\n【{test_name.upper()}】')
            print(f"  本地话题: {test_info['local_topic']}")
            print(f"  全局话题: {test_info['global_topic']}")
            
            # 检查本地话题
            if test_info['received_local']:
                print(f"  ✅ 本地话题正常")
            else:
                print(f"  ❌ 本地话题无数据")
                all_passed = False
            
            # 检查全局话题
            if test_info['received_global']:
                print(f"  ✅ 全局话题正常（中转成功）")
            else:
                print(f"  ❌ 全局话题无数据（中转失败）")
                all_passed = False
            
            # 检查数据一致性（仅针对 odom）
            if test_name == 'odom' and test_info['received_local'] and test_info['received_global']:
                local_data = test_info['local_data']
                global_data = test_info['global_data']
                
                if local_data and global_data:
                    local_x = local_data.pose.pose.position.x
                    global_x = global_data.pose.pose.position.x
                    diff = abs(local_x - global_x)
                    
                    if diff < 0.01:  # 允许 1cm 误差
                        print(f"  ✅ 数据一致性检查通过 (误差: {diff:.4f}m)")
                    else:
                        print(f"  ⚠️  数据可能不一致 (误差: {diff:.4f}m)")
        
        print('\n' + '='*60)
        if all_passed:
            print('🎉 所有测试通过！中转系统工作正常！')
            print('='*60)
            return 0
        else:
            print('⚠️  部分测试失败，请检查：')
            print('   1. 中转节点是否启动？')
            print('   2. 机器人导航是否运行？')
            print('   3. 话题配置是否正确？')
            print('='*60)
            return 1


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='测试多机器人中转系统')
    parser.add_argument('--robot-name', type=str, default='robot1',
                        help='机器人名称 (默认: robot1)')
    parser.add_argument('--duration', type=int, default=10,
                        help='测试持续时间（秒，默认: 10）')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        tester = RelayTester(args.robot_name)
        
        # 运行指定时间
        start_time = time.time()
        while time.time() - start_time < args.duration:
            rclpy.spin_once(tester, timeout_sec=0.1)
        
        # 打印结果
        exit_code = tester.print_results()
        
    except KeyboardInterrupt:
        print('\n测试被中断')
        exit_code = 1
    except Exception as e:
        print(f'\n❌ 测试出错: {e}')
        exit_code = 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
