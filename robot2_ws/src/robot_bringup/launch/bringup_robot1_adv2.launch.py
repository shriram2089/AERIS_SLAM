from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(robot_name='robot2'):
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    mapper_params = os.path.join(robot_bringup_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_params = os.path.join(robot_bringup_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # 🌟 Namespace wrapper for this robot
        PushRosNamespace(robot_name),

        # 🟡 YDLIDAR driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ydlidar_ros2_driver'),
                    'launch',
                    'ydlidar_launch_view2.py'
                )
            ),
        ),

        # 🟢 Serial odometry node
        Node(
            package='serial_odom',
            executable='serial_odom_new',
            name='serial_odom',
            output='screen',
        ),

        # 🔵 SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[mapper_params],
            output='screen',
        ),

        # 🟣 Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                )
            ),
            launch_arguments={
                'use_namespace': 'true',  # Let Nav2 respect PushRosNamespace
                'params_file': nav2_params,
                'use_rviz': 'true'
            }.items()
        ),
    


        # 🔴 Custom explorer node
        # Node(
        #     package='custom_explorer',
        #     executable='adv_explorer',
        #     name='adv_explorer',
        #     # namespace='robot1',
        #     output='screen',
        # ),
    ])

