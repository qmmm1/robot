from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')
    
    return LaunchDescription([
        # Simulation environment with random start position
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_dir,
                    'launch',
                    'sim_env_random.launch.py'
                ])
            ])
        ),
        
        # Phase 1 navigation system (no soft obstacles)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_dir,
                    'launch',
                    'phase1_to_start.launch.py'
                ])
            ])
        ),
        
        # RViz visualization
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
