import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_butlerbot_rmf_gazebo = FindPackageShare('butlerbot_rmf_gazebo').find('butlerbot_rmf_gazebo')
    pkg_gazebo = FindPackageShare('gazebo_ros').find('gazebo_ros')

    # File paths
    default_world_path = os.path.join(pkg_butlerbot_rmf_gazebo, 'worlds', 'cafe.world')
    models_path = os.path.join(pkg_butlerbot_rmf_gazebo, 'models')

    # Launch configuration variables with default values 
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    verbose = LaunchConfiguration('verbose') 
    

    declare_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use gazebo clock'
        ),
        DeclareLaunchArgument(
            name='world',
            default_value=default_world_path, 
            description='Absolute path of gazebo WORLD file'
        ),
        DeclareLaunchArgument(  
            name='verbose',
            default_value='true',
            description='Set "true" to increase messages written to terminal.'
        ),
    ]

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + pkg_butlerbot_rmf_gazebo + '/share' + ':' + models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  pkg_butlerbot_rmf_gazebo + "/share" + ':' + models_path

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))


    # Open simulation environment
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py'))
    )


    return LaunchDescription(
        declare_arguments + [
            start_gazebo
    ])
