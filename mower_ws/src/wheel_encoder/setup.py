from setuptools import setup
import os
from glob import glob

package_name = 'wheel_encoder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.drivers'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samyb',
    maintainer_email='samyb@todo.todo',
    description='Wheel odometry driver and publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odom_node = wheel_encoder.wheel_odom_node:main',
        ],
    },
)
