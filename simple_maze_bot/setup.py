import os
from setuptools import setup

package_name = 'simple_maze_bot'

# 辅助函数：递归收集资源文件
# 只要你的 maps/config/launch 文件夹在根目录下，它就能自动把所有子文件找出来
def collect_data_files(base_dir):
    data_files = []
    if not os.path.exists(base_dir):
        return []
    
    for root, dirs, files in os.walk(base_dir):
        if files:
            # 安装目标路径 = share/包名/相对路径
            destination = os.path.join('share', package_name, root)
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((destination, file_list))
    return data_files

setup(
    name=package_name,
    version='0.0.1',
    
    # =========================================================
    # 1. 关键修改：因为你的 scripts 目录下有 __init__.py
    #    所以它是 simple_maze_bot 的子包，建议显式列出
    # =========================================================
    packages=[package_name, package_name + '.scripts'],
    
    data_files=[
        # ROS 2 标准注册
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
    # =========================================================
    # 2. 关键修改：一次性打包 maps, config, launch 三个文件夹
    # =========================================================
    ] + collect_data_files('maps') + collect_data_files('config') + collect_data_files('launch'),
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yahboom',
    maintainer_email='yahboom@todo.todo',
    description='Simple maze bot navigation task',
    license='Apache-2.0',
    
    # =========================================================
    # 3. 关键修改：入口点路径必须匹配你的 __init__.py 结构
    #    格式：'命令 = 包名.子包名.文件名:函数名'
    # =========================================================
    entry_points={
        'console_scripts': [
            'go_to_start = simple_maze_bot.scripts.go_to_start:main',
            'start_maze_task = simple_maze_bot.scripts.start_maze_task:main',
        ],
    },
)
