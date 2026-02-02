import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    package_name = 'rover_navigation'
    pkg_share = get_package_share_directory(package_name)
    
    # Include Ignition Gazebo launch
    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'launch_ignition.launch.py'])
        ]),
        launch_arguments={
            'world_file': 'cafe.world'
        }.items()
    )
    
    # SLAM Toolbox
    slam_params_file = PathJoinSubstitution([pkg_share, 'config', 'mapper_params_online_async.yaml'])
    
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': True}
        ],
        output='screen'
    )
    
    # RViz
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'view_robot.rviz'])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        ignition_launch,
        slam_toolbox,
        rviz
    ])
