from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Initial Robot Pose for Phase 2 ---
    start_x = 1.58
    start_y = 0.15
    start_yaw = 0.0

    print(f"Robot starting at START point: x={start_x}, y={start_y}, yaw={start_yaw}")

    # --- Robot Description ---
    urdf_file_path = os.path.join(pkg_dir, 'urdf', 'MicroROS.urdf')
    if not os.path.exists(urdf_file_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_file_path}")
    
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        # Gazebo simulation environment
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
                'gui': 'true'
            }.items()
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description
                }
            ],
            output='screen'
        ),

        # Spawn entity at specified START position
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'micro_ros_bot',
                '-topic', 'robot_description',
                '-x', str(start_x),
                '-y', str(start_y),
                '-z', '0.05',
                '-Y', str(start_yaw)
            ],
            output='screen'
        ),
    ])
