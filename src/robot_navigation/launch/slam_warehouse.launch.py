#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument, EmitEvent, RegisterEventHandler#,GroupAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
# from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_dir = get_package_share_directory("robot_navigation")

    use_rviz = LaunchConfiguration("use_rviz")

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="If should launch rviz or not"
    )
    
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch_dir = os.path.join(slam_toolbox_dir,"launch")

    start_world_and_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_dir,"/robot_warehouse.launch.py"]),
        launch_arguments={
            "use_rviz":use_rviz
        }.items()
    )

    start_slam_toolbox_online_async_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_launch_dir,"/online_async_launch.py"])
    )

    
    ld = LaunchDescription()

    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(start_world_and_navigation_cmd)
    ld.add_action(start_slam_toolbox_online_async_cmd)
    
    return ld