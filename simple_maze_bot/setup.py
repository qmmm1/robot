from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simple_maze_bot'

setup(
    name=package_name,
    version='0.1.0',
    # Automatically find Python packages in the directory
    packages=find_packages(exclude=['test']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        
        # Include all configuration files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        
        # Include all map files
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
        
        # Include all URDF files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        
        # Include all Gazebo world files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Simple maze bot package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_to_start = simple_maze_bot.go_to_start:main',
            'start_maze_task = simple_maze_bot.start_maze_task:main',
            'test_complete_simulation = simple_maze_bot.test_complete_simulation:main',
        ],
    },
)
