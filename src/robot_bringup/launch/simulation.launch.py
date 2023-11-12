#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    # warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')
    world_bringup =  get_package_share_directory('robot_world')
    world_bringup_launch_dir = os.path.join(world_bringup, "launch")
    description_dir = get_package_share_directory("robot_description")
    bringup_dir = get_package_share_directory("robot_bringup")

    vision_pkg_share = get_package_share_directory("robot_vision")

    # warehouse_world_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([warehouse_launch_path, '/small_warehouse.launch.py'])
    # )

    world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([world_bringup_launch_dir, "/world_bringup.launch.py"])
    )

    urdf = os.path.join(
        description_dir,
        'urdf',
        "model.urdf")

    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', "my_robot",
            '-file', os.path.join(description_dir, 'models','model.sdf'),
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])
    
    start_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf])
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': os.path.join(warehouse_pkg_dir,"maps","005","map.yaml"),
            'use_sim_time': "true",
            "params_file": os.path.join(bringup_dir, "params", "nav2_params.yaml")}.items(),
    )

    start_aruco_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vision_pkg_share,"/ArucoDetection.launch.py"])
    )

    start_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    ld = LaunchDescription()
    # ld.add_action(start_rviz)
    ld.add_action(start_aruco_detection)
    ld.add_action(world_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(start_state_publisher)
    ld.add_action(start_nav2)
    return ld