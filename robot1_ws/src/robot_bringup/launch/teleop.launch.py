from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joystick node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0'}]
        ),

        # Teleop twist joy node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            # parameters=['/home/pi/teleop_config/teleop_joy2.yaml'],
            parameters=[{
                'axis_linear.x': 1,    # Left stick up/down
                'axis_angular.yaw': 0, # Left stick left/right
                'scale_linear.x': 0.5,
                'scale_angular.yaw': 0.5,
                'enable_button': 9
            }],
            remappings=[('/cmd_vel', '/robot1/cmd_vel')]
        )
    ])
