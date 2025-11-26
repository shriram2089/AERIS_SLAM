from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import launch_ros
import sys

def generate_launch_description():
    return LaunchDescription([
        # map → odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        # odom → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
        # base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_broadcaster',
            arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        )
    ])

def main(args=None):
    """Main entry point for running the launch file directly."""
    launch_service = launch.LaunchService(argv=args)
    ld = generate_launch_description()
    launch_service.include_launch_description(ld)
    return launch_service.run()

if __name__ == '__main__':
    sys.exit(main())
