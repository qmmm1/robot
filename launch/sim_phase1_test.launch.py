from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')
    
    # 加载 MicroROS.urdf 文件内容
    robot_description_content = Command(['xacro ', PathJoinSubstitution([pkg_dir, 'urdf', 'MicroROS.urdf'])])
    
    return LaunchDescription([
        # 使用随机位置环境
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_dir,
                    'launch',
                    'sim_env_random.launch.py'
                ])
            ])
        ),
        
        # 启动 Phase 1 导航
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_dir,
                    'launch',
                    'phase1_to_start.launch.py'
                ])
            ])
        ),
        
        # 添加 robot_state_publisher 节点以发布机器人的 URDF 描述
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(robot_description_content, value_type=str)
            }]
        ),
        
        # 如果需要在 Gazebo 中显示机器人模型，则添加 spawn_entity 节点
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'micro_ros_robot'],
            output='screen'
        )
    ])