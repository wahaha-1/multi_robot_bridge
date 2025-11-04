from setuptools import setup
import os
from glob import glob

package_name = 'multi_robot_relay'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='Multi-robot communication relay hub for topic namespace isolation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topic_relay = multi_robot_relay.topic_relay_node:main',
            'tf_relay = multi_robot_relay.tf_relay_node:main',
            'robot_registry = multi_robot_relay.robot_registry_node:main',
            'coordinate_transformer = multi_robot_relay.coordinate_transformer_node:main',
        ],
    },
)
