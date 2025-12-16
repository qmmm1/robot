# launch/phase2_maze.launch.py

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Map Server: 加载完整地图（含软障碍）
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(pkg_dir, 'maps', 'maze_original.yaml')}],
        remappings=[('/map', '/maze_full')]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'nav2_params_phase2.yaml')]
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'nav2_params_phase2.yaml')]
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'nav2_params_phase2.yaml')]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'nav2_params_phase2.yaml')]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'bt_navigator'
            ]}
        ]
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        map_server_node,
        amcl_node,
        planner_node,
        controller_node,
        bt_navigator_node,
        lifecycle_manager_node,
    ])