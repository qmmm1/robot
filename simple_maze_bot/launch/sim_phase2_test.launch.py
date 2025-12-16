from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')

    # 加载 URDF 文件（使用 xacro 兼容处理；若为纯 URDF，xacro 也能直接输出）
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_dir, 'urdf', 'MicroROS.urdf'])
    ])

    return LaunchDescription([
        # 使用在 START 点生成的环境
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_dir,
                    'launch',
                    'sim_env_at_start.launch.py'
                ])
            ])
        ),

        # 启动 Phase 2 导航
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_dir,
                    'launch',
                    'phase2_maze.launch.py'
                ])
            ])
        ),
    ])