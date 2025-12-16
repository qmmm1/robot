from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import random

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_maze_bot')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # ========================================================================
    # 🔑 关键修复开始：设置 GAZEBO_MODEL_PATH
    # ========================================================================
    # 1. 获取存放 STL 文件的那个包的安装路径
    description_package_name = 'yahboomcar_description'
    install_dir = get_package_prefix(description_package_name)

    # 2. 拼接出 share 目录的路径
    # 解释：你的 STL 在 install/yahboomcar_description/share/yahboomcar_description/meshes
    # Gazebo 需要指向 install/yahboomcar_description/share 这一层
    model_path = os.path.join(install_dir, 'share')

    # 3. 如果系统里本来就有这个变量，就加在后面；如果没有，就新建
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']

    # 打印一下路径，方便你在终端确认是否生效
    print(f"Set GAZEBO_MODEL_PATH to: {model_path}")
    # ========================================================================
    # 🔑 关键修复结束
    # ========================================================================


    # 随机生成机器人初始位置（用于 Phase 1）
    start_x = random.uniform(0.45, 1.75)
    start_y = random.uniform(0.45, 1.75)
    start_yaw = random.uniform(0.0, 2 * 3.14159)
    
    print(f"Robot starting at: x={start_x:.2f}, y={start_y:.2f}, yaw={start_yaw:.2f}")

    # 直接读取 MicroROS.urdf
    urdf_file_path = os.path.join(pkg_dir, 'urdf', 'MicroROS.urdf')
    if not os.path.exists(urdf_file_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_file_path}")
    
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        # 4. 必须先把环境变量注入进去
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),

        # 启动 Gazebo
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

        # 发布 robot_description
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

        # 在随机位置 spawn 小车
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'micro_ros_bot',
                '-topic', 'robot_description',
                '-x', str(start_x),
                '-y', str(start_y),
                '-z', '0.05',
                '-Y', str(start_yaw)
            ],
            output='screen'
        ),
    ])
