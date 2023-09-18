#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument, EmitEvent, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node #SetParameter
# from launch_ros.descriptions import ParameterFile
# from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    pkg_dir = get_package_share_directory("robot_behavior")
    description_dir = get_package_share_directory("robot_description")
    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')
    bringup_dir = get_package_share_directory("nav2_bringup")
    bringup_launch_dir = os.path.join(bringup_dir,"launch")
    navigation_dir = get_package_share_directory("robot_navigation")

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    params_file = LaunchConfiguration('params_file')

    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    robot_name = LaunchConfiguration('robot_name', default="my_robot")
    robot_sdf = LaunchConfiguration('robot_sdf')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(navigation_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(description_dir, 'models','model.sdf'),
        description='Full path to robot sdf file to spawn the robot in gazebo')
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='aic3_robot',
        description='name of the robot')
    
    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_dir,"rviz","robot.rviz"),
        description='rviz config file name')
    
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="If should launch rviz or not"
    )
    
    
    urdf = os.path.join(description_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # set_envs = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='')


    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': robot_description}],
        remappings=remappings)

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])


    warehouse_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([warehouse_launch_path, '/small_warehouse_launch.py'])
    )

    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir,"/navigation_launch.py"]),
        launch_arguments={
            "params_file": params_file,
            "use_namespace":"false",
            "namespace":"",
            'use_lifecycle_mgr': 'false',
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    start_rviz_cmd = Node(
       condition=IfCondition(use_rviz),
       package='rviz2',
       executable='rviz2',
       arguments=['-d', rviz_config_file],
       output='screen')
    
    exit_event_handler = RegisterEventHandler(
        condition=IfCondition(use_rviz),
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    
    ld = LaunchDescription()



    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_rviz_config_file)
    ld.add_action(declare_use_rviz)

    # ld.add_action(set_envs)
    ld.add_action(warehouse_world_cmd)
    
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    # ld.add_action(start_navigation_cmd)
    # ld.add_action(start_slam_toolbox_online_async_cmd)
    
    ld.add_action(start_rviz_cmd)
    ld.add_action(exit_event_handler)
   
    return ld