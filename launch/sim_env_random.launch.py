# launch/sim_env_random.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import random

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 随机生成机器人初始位置（用于 Phase 1）
    start_x = random.uniform(0.45, 1.75)
    start_y = random.uniform(0.45, 1.75)
    start_yaw = random.uniform(0, 2 * 3.14159)
    
    print(f"Robot starting at: x={start_x:.2f}, y={start_y:.2f}, yaw={start_yaw:.2f}")
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([pkg_dir, 'worlds', 'maze_environment.world']),
                'verbose': 'false',
                'gui': 'true'
            }.items()
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3',
                '-topic', 'robot_description',
                '-x', str(start_x),
                '-y', str(start_y),
                '-z', '0.01',
                '-Y', str(start_yaw)
            ],
            output='screen'
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_frequency': 50.0
            }],
            remappings=[('/joint_states', 'turtlebot3/joint_states')]
        ),
    ])