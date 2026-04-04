from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mower_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'shapely',  # For polygon operations
        'websockets',  # For lawnbot_motors WebSocket client
    ],
    zip_safe=True,
    maintainer='Capstone Team',
    maintainer_email='capstone@example.com',
    description='Navigation for autonomous lawn mower',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_planner_node = mower_navigation.coverage_planner_node:main',
            'motor_controller_node = mower_navigation.motor_controller_node:main',
            'obstacle_handler_node = mower_navigation.obstacle_handler_node:main',
            'navigation_controller_node = mower_navigation.navigation_controller_node:main',
        ],
    },
)
