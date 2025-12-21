from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import random

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # --- Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    # --- Random Robot Start Pose ---
    start_x = random.uniform(0.45, 1.75)
    start_y = random.uniform(0.45, 1.75)
    start_yaw = random.uniform(0.0, 2 * 3.14159)
    
    print(f"Robot starting at: x={start_x:.2f}, y={start_y:.2f}, yaw={start_yaw:.2f}")

    # --- Gazebo Model Path Setup ---
    description_package_name = 'yahboomcar_description'
    install_dir = get_package_prefix(description_package_name)
    model_path = os.path.join(install_dir, 'share')
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']
    
    print(f"Set GAZEBO_MODEL_PATH to: {model_path}")

    # --- Robot Description ---
    urdf_file_path = os.path.join(pkg_dir, 'urdf', 'MicroROS.urdf')
    if not os.path.exists(urdf_file_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_file_path}")
    
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        use_sim_time_arg,
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        
        # Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ),
            launch_arguments={
                'world': PathJoinSubstitution([pkg_dir, 'worlds', 'maze_environment.world']),
                'verbose': 'false',
                'gui': 'true',
                'use_sim_time': use_sim_time
            }.items()
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }],
            output='screen'
        ),

        # Spawn entity at random pose
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=[
                '-entity', 'micro_ros_bot',
                '-topic', 'robot_description',
                '-x', str(start_x),
                '-y', str(start_y),
                '-z', '0.05',
                '-Y', str(start_yaw),
                '-timeout', '30'
            ],
            output='screen'
        ),
    ])
