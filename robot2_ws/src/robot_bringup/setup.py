from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Unified bringup package for robot1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['laser_to_pcl = robot_bringup.laser_to_pcl:main',
        'mrg_relay = robot_bringup.mrg_relay_node:main',
        'static_tf = robot_bringup.static_tf:main',
        
        'footprint_marker = robot_bringup.footprint_marker2:main',
        'joy_to_gripper = robot_bringup.joy_to_gripper:main',
        '3d_pcl_visualizer = robot_bringup.3d_pcl_visualizer:main',
        # '3d_pcl_visualizer = robot_bringup.lidar3d_processed:main',
        
        '3d_pcl_visualizer2 = robot_bringup.3d_pcl_visualizer2:main',
        '3d_pcl_visualizer3 = robot_bringup.3d_pcl_visualizer3:main',
        
        
        'gripper_lidar_control = robot_bringup.gripper_lidar_control:main',
        'gripper_node = robot_bringup.grip_node:main',
        
        'yolo = robot_bringup.yolo:main',
        'yolo_light = robot_bringup.yolo_light:main',
        'human_detect = robot_bringup.human_detect:main',
        'object_detect = robot_bringup.object_detect:main',
        'object_detect_final = robot_bringup.object_detect_final:main',
        
        'object_detect2 = robot_bringup.object_detect2:main',
        
        
        
        
        
        
        ],
        
    },
)

