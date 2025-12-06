# launch/sim_complete_test.launch.py
"""
完整两阶段测试的仿真启动文件：
- 阶段1：机器人从随机位置导航到 START 点 (0.22, 1.65)
- 阶段2：从 START 点避开软障碍导航到终点 (1.58, 1.65)

注意：此 launch 文件仅启动仿真环境和 Phase 1 导航系统。
Phase 2 的地图切换和行为由 test_complete_simulation.py 脚本控制。
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')
    
    return LaunchDescription([
        # 启动带随机起始位置的仿真环境（Phase 1 起点）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_dir,
                    'launch',
                    'sim_env_random.launch.py'
                ])
            ])
        ),
        
        # 启动 Phase 1 导航系统（使用无软障碍地图）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_dir,
                    'launch',
                    'phase1_to_start.launch.py'
                ])
            ])
        ),
        
        # 可选：启动 RViz 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            ])],
            output='screen'
        ),
    ])