#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    rviz_config_file = os.path.join(share_dir, 'config','ydlidar_2.rviz')
    parameter_file = LaunchConfiguration('params_file')

    # Declare launch argument for YAML parameter file
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'X2.yaml'),
        description='File path to the ROS2 parameters file to use.'
    )

    # YDLIDAR driver node (Best Effort QoS)
    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='/',
        parameters=[
            parameter_file,
            {
                'frequency': 5.0,
                'scan_qos_overrides': {  # this sets Best Effort QoS
                    '/scan': {
                        'reliability': 'best_effort',
                        'durability': 'volatile'
                    }
                }
            }
        ],
        remappings=[
            ('/scan', '/robot2/scan')
        ]
    )

    # Static transform for LIDAR
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=[
            '0', '0', '0.02',  # translation
            '0', '0', '0', '1', # rotation
            'base_link',        # parent frame
            'laser_frame'       # child frame
        ],
    )

    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        # tf2_node,
        rviz2_node,
    ])
