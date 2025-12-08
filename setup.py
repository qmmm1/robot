# setup.py

import os
from glob import glob
from setuptools import setup

package_name = 'simple_maze_bot'

def collect_data_files(base_dir):
    data_files = []
    for root, dirs, files in os.walk(base_dir):
        if files:
            # 过滤掉 __pycache__ 和 .pyc 文件
            filtered_files = [
                os.path.join(root, f) for f in files
                if not f.endswith('.pyc') and '__pycache__' not in root
            ]
            if filtered_files:
                rel_path = os.path.relpath(root, '.')
                data_files.append((f'share/{package_name}/{rel_path}', filtered_files))
    return data_files

setup(
    name=package_name,
    version='0.1.0',
    # ✅ 只包含真正的 Python 模块
    packages=[package_name, package_name + '.scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + collect_data_files('maps') 
      + collect_data_files('config') 
      + collect_data_files('launch')
      + collect_data_files('urdf')
      + collect_data_files('worlds'),  # ✅ launch 仅作为数据文件
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Simple maze bot with two-phase Nav2 navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go_to_start = simple_maze_bot.scripts.go_to_start:main',
            'start_maze_task = simple_maze_bot.scripts.start_maze_task:main',
            'test_complete_simulation = simple_maze_bot.scripts.test_complete_simulation:main',
        ],
    },
)