# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

"""
Hardware-ready Nav2 launch file for a single TurtleBot3.
Removes Gazebo and sets use_sim_time to False.
Includes topic remappings for hardware.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition

def generate_launch_description():
    # Directories
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    map_merge_dir = get_package_share_directory("multirobot_map_merge")
    launch_dir_map_merge = os.path.join(map_merge_dir, "launch", "tb3_simulation")

    # Launch configurations
    namespace = LaunchConfiguration("namespace")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    slam_toolbox = LaunchConfiguration("slam_toolbox")
    slam_gmapping = LaunchConfiguration("slam_gmapping")
    params_file = LaunchConfiguration("params_file")

    # LaunchDescription
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument("namespace", default_value="robot1", description="Robot namespace"))
    ld.add_action(DeclareLaunchArgument("autostart", default_value="true", description="Automatically startup the Nav2 stack"))
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="False", description="Start RViz"))
    ld.add_action(DeclareLaunchArgument("rviz_config",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_namespaced_view.rviz"),
        description="Path to RViz config"))
    ld.add_action(DeclareLaunchArgument("use_robot_state_pub", default_value="True", description="Start robot_state_publisher"))
    ld.add_action(DeclareLaunchArgument("slam_toolbox", default_value="False", description="Enable SLAM Toolbox"))
    ld.add_action(DeclareLaunchArgument("slam_gmapping", default_value="True", description="Disable GMapping"))
    ld.add_action(DeclareLaunchArgument("params_file",
        default_value=os.path.join(launch_dir_map_merge, "config", "nav2_multirobot_params_1.yaml"),
        description="ROS2 params file for the robot"))

    # Robot state publisher (optional)
    # urdf_file = os.path.join(bringup_dir, "urdf", "turtlebot3_waffle.urdf")
    
    tb3_desc_dir = get_package_share_directory("turtlebot3_description")
    urdf_file = os.path.join(tb3_desc_dir, "urdf", "turtlebot3_waffle.urdf")    

    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            name="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": False}],
            arguments=[urdf_file],
            condition=IfCondition(use_robot_state_pub),
        )
    )

    # Include RViz if requested
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
            condition=IfCondition(use_rviz),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": "True",
                "rviz_config": rviz_config_file,
            }.items(),
        )
    )

    # Nav2 bringup for hardware with SLAM
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir_map_merge, "bringup_hw_launch.py")),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": "True",
                "slam": "True",
                "slam_toolbox": slam_toolbox,
                "slam_gmapping": slam_gmapping,
                "map": "",  # empty for live mapping
                "use_sim_time": "False",
                "params_file": params_file,
                "autostart": autostart,
            }.items(),
        )
    )

    # Logging
    ld.add_action(LogInfo(msg=["Launching Nav2 stack on hardware with SLAM"]))

    return ld


\