#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = "robot_world"
    world_dir = get_package_share_directory(pkg_name)


    pose = {'x': LaunchConfiguration('x_pose', default='-0.50'),
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
            '-entity', "aruco_03",
            '-file', os.path.join(world_dir, 'models',"arucos",f"marker_{2:04d}",'model.sdf'),
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

    # aruco_spawn_cmds = []

    start_aruco_tf_publisher = Node(
        package=pkg_name,
        executable="aruco_tf_static.py",
        output="screen"
    )
    
    # i = 2
    # marker_spawn =  Node(
    #         package='gazebo_ros',
    #         executable='spawn_entity.py',
    #         output='screen',
    #         arguments=[
    #             '-entity', 'markera',
    #             '-file', os.path.join(world_dir,'models','arucos',"markera",'model.sdf'),
    #             '-x', f"{float(i//14)-0.5}", '-y', f"{float(i%14)}", '-z', '4.00',
    #             '-R', "0.0", "-P", '1.57079632679', "-Y", "0.0"
    #         ]
        # )

    ld = LaunchDescription()
    width = 25
    # for i in range(200):
    #     spawn = Node(
    #         package='gazebo_ros',
    #         executable='spawn_entity.py',
    #         output='screen',
    #         arguments=[
    #             '-entity', f'marker_{i:04d}',
    #             '-file', os.path.join(world_dir,'models','arucos',f"marker_{i:04d}",'model.sdf'),
    #             '-x', f"{float(i//width)-0.5-4}", '-y', f"{float(i%width)-10}", '-z', '4.00',
    #             '-R', "0.0", "-P", '1.57079632679', "-Y", "0.0"
    #         ]
    #     )
    #     # aruco_spawn_cmds.append(spawn)
    #     ld.add_action(spawn)
    # ld.add_action(start_gazebo_spawner_cmd)
    # ld.add_action(marker_spawn)
    # for cmd in aruco_spawn_cmds:
    #     ld.add_action(cmd)
    ld.add_action(start_aruco_tf_publisher)
    return ld