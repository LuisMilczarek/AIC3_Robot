#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument, EmitEvent, RegisterEventHandler,GroupAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
# from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node #SetParameter
# from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():



    # bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_dir = get_package_share_directory("robot_navigation")
    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    robot_world_pkg_dir = get_package_share_directory('robot_world')
    bringup_dir = get_package_share_directory("nav2_bringup")

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    use_rviz = LaunchConfiguration("use_rviz")
    map = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    lifecycle_nodes = ['map_server', 'amcl']

    param_substitutions = {
        'use_sim_time': "true",
        'yaml_filename': map}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites=param_substitutions,
        convert_types=True)
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="If should launch rviz or not"
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(robot_world_pkg_dir, "maps","amazon_warehouse.yaml"),
        description='rviz config file name')
    
    declare_params_file = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use')
    
    start_world_and_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_dir,"/robot_warehouse.launch.py"]),
        launch_arguments={
            "use_rviz":use_rviz
        }.items()
    )

    # launch_localization_cmd =IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([bringup_dir,"/launch",
    #                                                    '/localization_launch.py']),
    #         launch_arguments={'namespace': "",
    #                           "use_namespace":"false",
    #                           'map': map,
    #                           'use_sim_time': "true",
    #                           'autostart': "true",
    #                           'params_file': params_file,
    #                           'use_lifecycle_mgr': 'false'}.items())

    navigation_cmd = GroupAction([
        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     output='screen',
        #     parameters=[configured_params]),
    
        # Node(
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen',
        #     parameters=[configured_params]),
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_localization',
        #     output='screen',
        #     parameters=[{'use_sim_time': True},
        #                 {'autostart': True},
        #                 {'node_names': lifecycle_nodes}])
                        ])

    
    ld = LaunchDescription()

    ld.add_action(declare_map_file)
    ld.add_action(declare_params_file)
    ld.add_action(declare_use_rviz_cmd)    

    # ld.add_action(navigation_cmd)
    ld.add_action(start_world_and_navigation_cmd)
    # ld.add_action(map_server_cmd)
    # ld.add_action(amcl_cmd)
    # ld.add_action(lifecycle_manager)
    # ld.add_action(launch_localization_cmd)
    
    return ld