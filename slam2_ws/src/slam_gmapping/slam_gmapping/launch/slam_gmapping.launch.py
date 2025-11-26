from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    # Proper ROS remappings for your hardware
    remappings = [
        ("/scan", "/robot2/scan"),
        ("/map", "/robot2/map"),
        ("/odom", "/robot2/odom"),
        ("/tf", "/robot2/tf"),
        ("/tf_static", "/robot2/tf_static"),
    ]

    # Create the launch description and populate
    ld = LaunchDescription(
        [
            declare_namespace_cmd,
            declare_use_sim_time_cmd,
            Node(
                package="slam_gmapping",
                namespace=namespace,
                executable="slam_gmapping",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=remappings,
            ),
        ]
    )

    return ld
