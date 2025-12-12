from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 机器人在 START 点生成（用于 Phase 2）
    start_x = 1.58  # START 点 x 坐标
    start_y = 0.15  # START 点 y 坐标
    start_yaw = 0.0  # 朝向

    print(f"Robot starting at START point: x={start_x}, y={start_y}, yaw={start_yaw}")

    # 加载 MicroROS.urdf
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([pkg_dir, 'urdf', 'MicroROS.urdf'])
    ])

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

        # 发布 robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(robot_description_content, value_type=str)
            }],
            output='screen'
        ),
        
        # 在指定位置 spawn MicroROS 小车
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'micro_ros_bot',          # 实体名更改
                '-topic', 'robot_description',
                '-x', str(start_x),
                '-y', str(start_y),
                '-z', '0.05',
                '-Y', str(start_yaw)
            ],
            output='screen'
        ),
    ])