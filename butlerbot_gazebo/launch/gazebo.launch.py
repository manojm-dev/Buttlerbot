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
    pkg_share = FindPackageShare('butlerbot_gazebo').find('butlerbot_gazebo')
    description_share = FindPackageShare('butlerbot_description').find('butlerbot_description')
    gazebo_share = FindPackageShare('gazebo_ros').find('gazebo_ros')

    # Files paths 
    default_world_path = os.path.join(pkg_share, 'worlds/cafe.world')
    extra_models_path = os.path.join(pkg_share, 'models')

    # Launch configuration variables with default values 
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    verbose = LaunchConfiguration('verbose') 


    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + pkg_share + '/share' + ':' + extra_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  pkg_share + "/share" + ':' + extra_models_path

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))

    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
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

    # Robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_share, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time'    : use_sim_time,
                          'use_gazebo'      : 'true',
                          'use_gzsim'       : 'false',
                          }.items()
    )

    # Open simulation environment
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')), 
    )

    spawn_entity_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        name="spawn_entity",
        output='screen',
        arguments=[
            '-timeout', '120.0',
            '-topic', 'robot_description', 
            '-entity', 'butlerbot_1',
            '-z', '0.5', '-x', '9.0', '-y', '16.0',
            '-Y', '3.14'],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )


    return LaunchDescription(
        declare_arguments + [
            robot_state_publisher,
            start_gazebo,
            spawn_entity_node
        ] 
    )