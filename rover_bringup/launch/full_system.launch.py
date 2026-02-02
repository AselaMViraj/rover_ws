import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import TimerAction


def generate_launch_description():
    # Package references
    bringup_pkg = 'rover_bringup'
    simulation_pkg = 'rover_simulation'
    navigation_pkg = 'rover_navigation'
    description_pkg = 'rover_description'
    web_pkg = 'rover_web_interface'

    # Launch arguments
    world_file = LaunchConfiguration('world_file', default='cafe.world')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    slam = LaunchConfiguration('slam', default='true')
    explore = LaunchConfiguration('explore', default='false')
    localization = LaunchConfiguration('localization', default='false')
    map_file = LaunchConfiguration('map', default='')

    # Ignition Simulation with robot spawn and bridges
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(simulation_pkg), 'launch', 'launch_ignition.launch.py')),
        launch_arguments={
            'world_file': world_file,
            'slam': 'false' # We will launch SLAM separately or via exploration
        }.items()
    )

    # Autonomous Exploration (includes SLAM and Nav2)
    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(navigation_pkg), 'launch', 'exploration_launch.py')),
        condition=IfCondition(explore)
    )

    # Standard SLAM (if exploration is NOT selected)
    # We use a trick to only launch if slam=true AND explore=false
    # But for a master file, let's keep it simple:
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(navigation_pkg), 'launch', 'slam.launch.py')),
        condition=IfCondition(slam)
    )

    # Static Localization (AMCL + Map Server)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(navigation_pkg), 'launch', 'localization_launch.py')),
        condition=IfCondition(localization),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items()
    )

    # Static Navigation (Nav2 Servers)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(navigation_pkg), 'launch', 'navigation_launch.py')),
        condition=IfCondition(localization),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(description_pkg), 'config', 'view_robot.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        # Fix for GLSL error on some drivers - MERGE with existing env
        env={**dict(os.environ), 'MESA_GL_VERSION_OVERRIDE': '3.3'},
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value='cafe.world'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('explore', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),
        
        simulation,
        # Management Layer (Phase 3)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        Node(
            package='rover_navigation',
            executable='waypoint_manager.py',
            name='waypoint_manager',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        Node(
            package='rover_web_interface',
            executable='web_server.py',
            name='web_server',
            output='screen'
        ),

        # Give simulation a moment to stabilize
        TimerAction(period=2.0, actions=[
            slam_toolbox,
            localization_launch,
            navigation_launch,
            exploration,
            rviz
        ]),
    ])
