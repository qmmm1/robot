from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')

    # --- Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    x_arg = DeclareLaunchArgument('x', default_value='1.65', description='Initial x position')
    y_arg = DeclareLaunchArgument('y', default_value='1.22', description='Initial y position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='2.01', description='Initial yaw')

    # --- Environment Variables ---
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # --- Nodes ---
    
    # Static TF: map -> odom
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': os.path.join(pkg_dir, 'maps', 'maze_no_soft.yaml')
        }]
    )

    # AMCL Localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'set_initial_pose': True,
            'initial_pose.x': LaunchConfiguration('x'),
            'initial_pose.y': LaunchConfiguration('y'),
            'initial_pose.yaw': LaunchConfiguration('yaw')
        }]
    )

    # Planner Server
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'nav2_params_phase1.yaml')]
    )

    # Controller Server
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'nav2_params_phase1.yaml')]
    )

    # Behavior Server
    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'nav2_params_phase1.yaml')]
    )

    # BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'nav2_params_phase1.yaml')]
    )

    # Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        x_arg, y_arg, yaw_arg,
        stdout_linebuf_envvar,
        static_tf_publisher,
        map_server_node,
        amcl_node,
        planner_node,
        controller_node,
        behavior_node,
        bt_navigator_node,
        lifecycle_manager_node,
    ])
