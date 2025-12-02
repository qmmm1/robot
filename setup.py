# setup.py

import os
from glob import glob
from setuptools import setup

package_name = 'simple_maze_bot'

# 自动收集所有资源文件
def collect_data_files(base_dir):
    data_files = []
    for root, dirs, files in os.walk(base_dir):
        if files:
            rel_path = os.path.relpath(root, '.')
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((f'share/{package_name}/{rel_path}', file_list))
    return data_files

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.scripts', package_name + '.launch'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + collect_data_files('maps') + collect_data_files('config') + collect_data_files('launch'),
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
        ],
    },
)