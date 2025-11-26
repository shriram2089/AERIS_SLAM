#!/usr/bin/env python3
# bringup_hw_launch.py — fixed to always use your custom nav2 YAML inside map_merge/config

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Base directories
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    slam_gmapping_dir = get_package_share_directory("slam_gmapping")
    slam_gmapping_launch_dir = os.path.join(slam_gmapping_dir, "launch")

    map_merge_dir = get_package_share_directory("multirobot_map_merge")
    map_merge_launch_dir = os.path.join(map_merge_dir, "launch", "tb3_simulation")
    config_dir = os.path.join(map_merge_launch_dir, "config")

    # Custom params file path (✅ edit only this line if needed)
    # my_params_path = os.path.join(config_dir, "nav2_multirobot_params_2_final.yaml")
    my_params_path = "/home/pi/slam_ws/src/m-explore-ros2/map_merge/launch/tb3_simulation/config/nav2_multirobot_params_2_final.yaml"

    # Launch configurations
    namespace = LaunchConfiguration("namespace", default="robot2")
    use_namespace = LaunchConfiguration("use_namespace", default="true")
    slam = LaunchConfiguration("slam", default="True")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    slam_toolbox = LaunchConfiguration("slam_toolbox", default="False")
    slam_gmapping = LaunchConfiguration("slam_gmapping", default="True")
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")

    # Unbuffered logging
    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)

    # ───────────────────────────────────────────────
    # Launch Arguments
    # ───────────────────────────────────────────────
    ld.add_action(DeclareLaunchArgument(
        "namespace", default_value="robot2", description="Top-level namespace"))
    ld.add_action(DeclareLaunchArgument(
        "use_namespace", default_value="true", description="Use namespace?"))
    ld.add_action(DeclareLaunchArgument(
        "slam", default_value="True", description="Run SLAM?"))
    ld.add_action(DeclareLaunchArgument(
        "slam_toolbox", default_value="False", description="Use SLAM toolbox"))
    ld.add_action(DeclareLaunchArgument(
        "slam_gmapping", default_value="True", description="Use GMapping"))
    ld.add_action(DeclareLaunchArgument(
        "map", default_value="", description="Map yaml file"))
    
    # ✅ Load your file inside map_merge/config
    ld.add_action(DeclareLaunchArgument(
        "params_file",
        default_value=my_params_path,
        description="Custom Nav2 params file for robot2"
    ))

    ld.add_action(DeclareLaunchArgument(
        "autostart", default_value="true", description="Autostart Nav2 stack"))

    # Log which file is being used
    ld.add_action(LogInfo(msg=["🚀 Using Nav2 params file: ", my_params_path]))

    # ───────────────────────────────────────────────
    # NAVIGATION GROUP
    # ───────────────────────────────────────────────
    nav_group = GroupAction([
        PushRosNamespace(namespace),  # all nodes under robot2

        # SLAM Toolbox (optional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(map_merge_launch_dir, "slam_toolbox.py")),
            condition=IfCondition(PythonExpression([slam, " and ", slam_toolbox, " and not ", slam_gmapping])),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "namespace": namespace,
            }.items(),
        ),

        # Localization (only if not running SLAM)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "localization_launch.py")),
            condition=IfCondition(PythonExpression(["not ", slam])),
            launch_arguments={
                "namespace": namespace,
                "map": map_yaml_file,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
                "use_lifecycle_mgr": "false",
            }.items(),
        ),

        # Nav2 Stack (always loaded)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "navigation_launch.py")),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
                "use_lifecycle_mgr": "false",
                "map_subscribe_transient_local": "true",
            }.items(),
        ),
    ])

    # ───────────────────────────────────────────────
    # SLAM GMAPPING
    # ───────────────────────────────────────────────
    gmapping_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_gmapping_launch_dir, "slam_gmapping.launch.py")),
        condition=IfCondition(PythonExpression([slam, " and ", slam_gmapping, " and not ", slam_toolbox])),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "remap_scan": [namespace, "/scan"],
            "remap_map": [namespace, "/map"],
            "remap_odom": [namespace, "/odom"],
            "remap_tf": ["/tf", "/robot2/tf"],
            "remap_tf_static": ["/tf_static", "/robot2/tf_static"],
        }.items(),
    )

    ld.add_action(nav_group)
    ld.add_action(gmapping_cmd)
    
    init_pose = ExecuteProcess(
    cmd=[
        "ros2", "topic", "pub", "--once",
        "/robot2/initialpose",
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 1.5, y: 0.8, z: 0.0}, orientation: {z: 0.7071, w: 0.7071}}}}"
    ],
    output="screen",
)
    # ld.add_action(init_pose)

    return ld
