import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'maze_mouse'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cx',
    maintainer_email='cx@todo.todo',
    description='Micromouse maze solving with TurtleBot3',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
        entry_points={
        'console_scripts': [
            'open_loop = maze_mouse.open_loop:main',
            'p_control = maze_mouse.p_control:main',
            'scan_zones = maze_mouse.scan_zones:main',
            'wall_follower = maze_mouse.wall_follower:main',
            'pure_pursuit = maze_mouse.pure_pursuit:main',
            'wall_follower_adaptive = maze_mouse.wall_follower_adaptive:main',
        ],
    },
)
