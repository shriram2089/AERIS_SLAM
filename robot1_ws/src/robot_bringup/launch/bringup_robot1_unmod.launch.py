from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    mapper_params = os.path.join(robot_bringup_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_params = os.path.join(robot_bringup_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # 🟡 YDLIDAR driver with namespace
        PushRosNamespace('robot1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ydlidar_ros2_driver'),
                    'launch',
                    'ydlidar_launch_view.py'
                )
            ),
            launch_arguments={
                # 'namespace': 'robot1'
            }.items()
        ),

        🟢 Serial odometry node
        Node(
            package='serial_odom',
            executable='serial_odom_node',
            name='serial_odom',
            # namespace='robot1',
            output='screen',
        ),
        
        #   Node(
        #     package='serial_odom',
        #     executable='dd_serial_node',
        #     name='dd_serial',
        #     # namespace='robot1',
        #     output='screen',
        # ),

        # 🔵 SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            # namespace='robot1',
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
                # 'namespace': 'robot1',
                'use_namespace': 'false',
                'params_file': nav2_params,
                'use_rviz': 'true'
            }.items()
        ),

        # 🔴 Custom explorer node
        Node(
            package='custom_explorer',
            executable='explorer',
            name='explorer',
            # namespace='robot1',
            output='screen',
        ),
    ])

