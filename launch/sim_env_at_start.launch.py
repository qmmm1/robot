from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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

    # 机器人在 START 点生成（用于 Phase 2）
    start_x = 1.58   # START 点 x 坐标
    start_y = 0.15   # START 点 y 坐标
    start_yaw = 0.0  # 朝向（弧度）

    print(f"Robot starting at START point: x={start_x}, y={start_y}, yaw={start_yaw}")

    return LaunchDescription([
        model_arg,

        # 启动 Gazebo 仿真环境
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

        # 发布 robot_description 到 ROS 2 系统
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

        # 添加 base_footprint -> base_link 的静态 TF（Z=0.05 表示 base_link 高出地面 5cm）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),

        # 在指定起点位置 spawn MicroROS 小车
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'micro_ros_bot',
                '-topic', 'robot_description',
                '-x', str(start_x),
                '-y', str(start_y),
                '-z', '0.0',          # 注意：现在 spawn 到 z=0（因为 base_footprint 在地面）
                '-Y', str(start_yaw)
            ],
            output='screen'
        ),
    ])