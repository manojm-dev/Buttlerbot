import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Packages share directory
    pkg_butler_navigation = FindPackageShare('butlerbot_navigation').find('butlerbot_navigation')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup').find('nav2_bringup')

    # Files paths 
    default_param_file = os.path.join(pkg_butler_navigation, 'config/nav2_params.yaml')

    # Launch configuration variables with default values 
    use_sim_time = LaunchConfiguration('use_sim_time')
    param_file = LaunchConfiguration('params_file')


    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        DeclareLaunchArgument(  
            name='params_file',
            default_value=default_param_file,
            description='Navigation2 parameter file'
        ),
    ]

    # Robot state publisher
    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time'  : use_sim_time,
            'param_file'    : param_file
            }.items()
    )


    return LaunchDescription(
        declare_arguments + [
            start_navigation
        ] 
    )