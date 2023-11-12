#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory("robot_world")
    pkg_launch_dir = os.path.join(pkg_dir, "launch")

    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')

    warehouse_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([warehouse_launch_path, '/small_warehouse.launch.py'])
    )

    arucos_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_launch_dir,"/spawn_arucos.launch.py"])
    )
    
   

    ld = LaunchDescription()
    ld.add_action(warehouse_world_cmd)
    ld.add_action(arucos_cmd)
    return ld