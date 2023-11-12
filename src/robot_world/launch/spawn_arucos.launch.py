#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = "robot_world"
    world_dir = get_package_share_directory(pkg_name)


    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='4.00'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='1.57079632679'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', "aruco_01",
            '-file', os.path.join(world_dir, 'models',"arucos","marker",'model.sdf'),
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

    start_aruco_tf_publisher = Node(
        package=pkg_name,
        executable="aruco_tf_static.py",
        output="screen"
    )
    
   

    ld = LaunchDescription()
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(start_aruco_tf_publisher)
    return ld