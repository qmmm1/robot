from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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

    # 加载 MicroROS.urdf（使用 xacro 兼容处理）
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_dir, 'urdf', 'MicroROS.urdf'])
    ])

    return LaunchDescription([
        # 启动 Gazebo
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

        # 发布 robot_description（关键！Gazebo 通过 /robot_description 读取模型）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(robot_description_protected := robot_description_content, value_type=str)
            }],
            output='screen'
        ),

        # 在随机位置 spawn MicroROS 小车
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'micro_ros_bot',          # 实体名（不能重复）
                '-topic', 'robot_description',       # 从该 topic 读取 URDF
                '-x', str(start_x),
                '-y', str(start_y),
                '-z', '0.01',
                '-Y', str(start_yaw)
            ],
            output='screen'
        ),
    ])