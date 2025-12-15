from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import random

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')
    urdf_pkg_path = get_package_share_path('simple_maze_bot')
    default_model_path = urdf_pkg_path / 'urdf' / 'MicroROS.urdf'

    # 声明 model 参数，允许用户覆盖 URDF 路径
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot URDF file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model_path = LaunchConfiguration('model')

    # 使用 xacro 命令加载 URDF（兼容 .urdf 和 .xacro）
    robot_description = ParameterValue(
        Command(['xacro ', model_path]),
        value_type=str
    )

    # 随机生成机器人初始位置（用于 Phase 1）
    start_x = random.uniform(0.45, 1.75)
    start_y = random.uniform(0.45, 1.75)
    start_yaw = random.uniform(0.0, 2 * 3.14159)

    print(f"Robot starting at: x={start_x:.2f}, y={start_y:.2f}, yaw={start_yaw:.2f}")

    return LaunchDescription([
        model_arg,

        # 启动 Gazebo
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

        # 发布 robot_description
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

        # 添加 base_footprint -> base_link 的静态 TF（Z=0.05 表示 base_link 离地 5cm）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),

        # 在随机位置 spawn MicroROS 小车
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'micro_ros_bot',
                '-topic', 'robot_description',
                '-x', str(start_x),
                '-y', str(start_y),
                '-z', '0.05',      # base_link 离地高度，与 static_transform 一致
                '-Y', str(start_yaw)
            ],
            output='screen'
        ),
    ])