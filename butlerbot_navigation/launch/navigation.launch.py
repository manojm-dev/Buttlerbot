import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare('butlerbot_navigation').find('butlerbot_navigation')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup').find('nav2_bringup')

    # Files paths 
    default_param_file = os.path.join(pkg_share, 'config/nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz/navigation.rviz')

    # Launch configuration variables with default values 
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
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

        DeclareLaunchArgument(  
            name='use_rviz',
            default_value='true',
            description='Use Rviz Visualization tools'
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

    # Start RViz
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': True
        }],
        condition=IfCondition(use_rviz)
    )


    return LaunchDescription(
        declare_arguments + [
            start_navigation,
            start_rviz
        ] 
    )