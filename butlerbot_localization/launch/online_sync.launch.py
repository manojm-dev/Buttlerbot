import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directories
    pkg_share = FindPackageShare('butlerbot_localization').find('butlerbot_localization')
    slamtb_share = FindPackageShare('slam_toolbox').find('slam_toolbox')

    # File paths
    default_slam_config = os.path.join(pkg_share, 'config/mapper_params_online_sync.yaml')

    # Launch configuration variables with default values
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration with file paths
    slam_config = LaunchConfiguration('slam_config')

    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        DeclareLaunchArgument(
            name='slam_config',
            default_value=default_slam_config,
            description='Absolute path of ekf config file'
        ),
    ]

    start_slamtb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slamtb_share, 'launch', 'online_sync_launch.py')), 
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': use_sim_time
            }.items()
    )

    return LaunchDescription(
        declare_arguments + [
            start_slamtb
        ]
    )