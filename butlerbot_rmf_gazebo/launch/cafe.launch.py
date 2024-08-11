import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare('butlerbot_rmf_gazebo').find('butlerbot_rmf_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')
    failover_mode = LaunchConfiguration('failover_mode')

    declare_arguments = [

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use gazebo clock'
        ),
        
        DeclareLaunchArgument(
        name= 'failover_mode',
        default_value='false',
        description='Enable or disable failover mode'
        )
    ]


    launch_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'simulation.launch.py'))
    )
    
    return LaunchDescription(
        declare_arguments + [
            launch_world,

        ]
    )