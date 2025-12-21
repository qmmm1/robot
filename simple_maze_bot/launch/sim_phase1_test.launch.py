from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')
    
    # --- Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        use_sim_time_arg,
        
        # Step 1: Start Gazebo environment and robot immediately
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_dir,
                    'launch',
                    'sim_env_random.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),
        
        # Step 2: Delay navigation system start by 5s to ensure Gazebo is ready
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            pkg_dir,
                            'launch',
                            'phase1_to_start.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'x': '1.65',
                        'y': '1.22',
                        'yaw': '2.01'
                    }.items()
                )
            ]
        )
    ])
