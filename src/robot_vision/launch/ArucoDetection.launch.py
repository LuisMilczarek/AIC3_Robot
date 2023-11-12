#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    arucoDetection_cmd = Node(
        package="robot_vision",
        executable="aruco_recognition",
        output="screen"
    )
 
    ld = LaunchDescription()
    ld.add_action(arucoDetection_cmd)
    return ld