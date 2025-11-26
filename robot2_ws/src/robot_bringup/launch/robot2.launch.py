from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction
import os

def generate_launch_description():
    slam_params_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    )

    nav2_params_file = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'nav2_params_mod.yaml'
    )

    # First configure process
    configure_slam = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/robot2/slam_toolbox', 'configure'],
        output='screen'
    )

    # Then activate process (triggered after configure exits)
    activate_slam = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/robot2/slam_toolbox', 'activate'],
        output='screen'
    )

    return LaunchDescription([
        PushRosNamespace('robot2'),

        # Launch YDLIDAR driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ydlidar_ros2_driver'),
                    'launch',
                    'ydlidar_launch_view.py'
                )
            ),
            launch_arguments={'namespace': 'robot2'}.items()
        ),

        # Serial odometry node
        Node(
            package='serial_odom',
            executable='serial_odom_node3',
            name='serial_odom',
            output='screen',
            # remappings=[
            #     ('/tf', '/robot2/tf'),
            #     ('/tf_static', '/robot2/tf_static')
            # ]
        ),

        
        # # SLAM Toolbox lifecycle node
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='slam_toolbox',
        #     parameters=[
        #         slam_params_file,
        #         {
        #             'use_sim_time': False,
        #             'scan_queue_size': 200,
        #             'transform_publish_period': 0.02,
        #             'map_update_interval': 2.0,
        #             'autostart': False  # we control manually
        #         }
        #     ],
        #     remappings=[
        #         ('/scan', '/robot2/scan'),
        #         ('/odom', '/robot2/odom'),
        #         ('/map', '/robot2/map'),
        #     ],
        #     output='screen',
        # ),

        # # Configure then activate slam_toolbox in sequence
        # configure_slam,
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=configure_slam,
        #         on_exit=[activate_slam]
        #     )
        # ),

        # Other nodes
        Node(
            package='robot_bringup',
            executable='laser_to_pcl',
            name='laser_to_pcl',
            output='screen',
        ),
        Node(
            package='robot_bringup',
            executable='mrg_relay',
            name='mrg_relay',
            output='screen',
        ),
        Node(
            package='robot_bringup',
            executable='footprint_marker',
            name='fp_marker',
            output='screen',
        ),
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='base_to_scan',
    #         arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'scan'],
    #         namespace='robot2'
    #     ),
    
        TimerAction(
                    period=5.0,  # seconds
                    actions=[
                        Node(
                            package='robot_bringup',
                            executable='gripper_lidar_control',
                            name='gripper_lidar_control',
                            output='screen',
                        )
                    ]
                ),

        # Start 3d_pcl_visualizer 3 seconds after the previous node
        TimerAction(
            period=8.0,  # seconds from launch (5+3)
            actions=[
                Node(
                    package='robot_bringup',
                    executable='3d_pcl_visualizer',
                    name='three_d_pcl_visualizer',
                    output='screen',
                )
            ]
        ),
        
        
        
        #      TimerAction(
        #     period=10.0,  # seconds from launch (5+3)
        #     actions=[
        #         Node(
        #             package='robot_bringup',
        #             executable='object_detect_final',
        #             name='object_detect',
        #             output='screen',
        #         )
        #     ]
        # ),
    ])
