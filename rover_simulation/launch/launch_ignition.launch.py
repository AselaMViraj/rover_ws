import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    package_name = 'rover_simulation'
    
    # Launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='house.world',
        description='World file name (cafe.world, house.world, or obstacles.world)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Start SLAM'
    )
    
    # Get paths
    pkg_share = get_package_share_directory(package_name)
    world_path = PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world_file')])
    
    # Model paths - need actual strings for environment variable
    models_install_path = os.path.join(pkg_share, 'models')
    pkg_src_path = os.path.join(pkg_share.replace('/install/', '/src/'), 'models')
    
    # Set Ignition Gazebo resource path to include our models
    # Include both install and source paths (source for development with symlink-install)
    if os.path.exists(pkg_src_path):
        models_path_str = f"{models_install_path}:{pkg_src_path}"
    else:
        models_path_str = models_install_path
    
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_path_str
    )
    
    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([get_package_share_directory('rover_description'), 'launch', 'rsp.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'false'  # Use gazebo_control for Ignition
        }.items()
    )
    
    # Ignition Gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'articubot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )
    
    # Bridge for /scan topic
    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/articubot/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Bridge for /cmd_vel topic
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Bridge for /odom topic
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/articubot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Bridge for model pose (for TF)
    bridge_pose = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/articubot/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    

    
    # Bridge for /joint_states topic
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/model/articubot/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Bridge for /tf topic
    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Bridge for /clock topic
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )
    
    # SLAM Toolbox (optional) - Now references navigation package
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([get_package_share_directory('rover_navigation'), 'launch', 'slam.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=launch.conditions.IfCondition(LaunchConfiguration('slam'))
    )
    
    return LaunchDescription([
        world_file_arg,
        use_sim_time_arg,
        slam_arg,
        set_ign_resource_path,
        ignition_gazebo,
        
        # High-priority clock bridge (global namespace)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            output='screen'
        ),
        
        # Secondary bridge for sensors and status
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        
        # odom -> base_link transform publisher
        Node(
            package='rover_bringup',
            executable='odom_tf_publisher.py',
            name='odom_tf_publisher',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        
        rsp,
        spawn_robot,
        slam_toolbox
    ])
