from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')

    # --- Robot Description ---
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_dir, 'urdf', 'MicroROS.urdf'])
    ])

    return LaunchDescription([
        # Start simulation at the designated START point
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_dir,
                    'launch',
                    'sim_env_at_start.launch.py'
                ])
            ])
        ),

        # Start Phase 2 navigation system
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
