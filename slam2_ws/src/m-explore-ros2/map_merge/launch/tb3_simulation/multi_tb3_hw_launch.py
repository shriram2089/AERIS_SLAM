# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

"""
Hardware-ready launch file for multi-robot Nav2 with m-explore-ros2 package.
This launch file removes Gazebo simulation and sets up hardware with topic remapping.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
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

    # Robot names
    robots = ["robot2"]

    # Launch configurations
    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    log_settings = LaunchConfiguration("log_settings", default="true")

    # Declare launch arguments
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true", description="Automatically startup the stacks"
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_namespaced_view.rviz"),
        description="Full path to the RVIZ config file to use.",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    # Declare parameter files for hardware
    declare_robot1_params_file_cmd = DeclareLaunchArgument(
        "robot1_params_file",
        default_value=os.path.join(
            launch_dir_map_merge, "config", "nav2_multirobot_params_1.yaml"
        ),
        description="Full path to the ROS2 parameters file for robot1"
    )
    declare_robot2_params_file_cmd = DeclareLaunchArgument(
        "robot2_params_file",
        default_value=os.path.join(
            launch_dir_map_merge, "config", "nav2_multirobot_params_2.yaml"
        ),
        description="Full path to the ROS2 parameters file for robot2"
    )

    # SLAM configurations
    slam_toolbox = LaunchConfiguration("slam_toolbox")
    slam_gmapping = LaunchConfiguration("slam_gmapping")
    declare_slam_toolbox_cmd = DeclareLaunchArgument(
        "slam_toolbox", default_value="False", description="Enable SLAM Toolbox"
    )
    declare_slam_gmapping_cmd = DeclareLaunchArgument(
        "slam_gmapping", default_value="True", description="Disable GMapping"
    )

    # Hardware Nav2 launch groups
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(f"{robot}_params_file")

        group = GroupAction(
            [
                # RViz
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        "namespace": TextSubstitution(text=robot),
                        "use_namespace": "True",
                        "rviz_config": rviz_config_file,
                    }.items(),
                ),
                # Hardware Nav2 bringup
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir_map_merge, "tb3_hw_launch.py")),
                    launch_arguments={
                        "namespace": robot,
                        "use_namespace": "True",
                        "map": "",  # optional for live mapping
                        "use_sim_time": "False",
                        "params_file": params_file,
                        "autostart": autostart,
                        "slam": "True",
                        "slam_toolbox": slam_toolbox,
                        "slam_gmapping": slam_gmapping,
                        "use_robot_state_pub": use_robot_state_pub,
                    }.items(),
                ),
                # Logging
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[f"Launching {robot} on hardware"]
                ),
            ]
        )
    nav_instances_cmds.append(group)

    # Create launch description
    ld = LaunchDescription()

    # Add all launch arguments
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_robot1_params_file_cmd)
    ld.add_action(declare_robot2_params_file_cmd)
    ld.add_action(declare_slam_toolbox_cmd)
    ld.add_action(declare_slam_gmapping_cmd)

    # Add hardware Nav2 groups
    for nav_instance in nav_instances_cmds:
        ld.add_action(nav_instance)

    return ld
