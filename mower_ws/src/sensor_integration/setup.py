from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sensor_integration'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Capstone Team',
    maintainer_email='your-email@example.com',
    description='Sensor integration for autonomous lawn mower',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = sensor_integration.imu_node:main',
            'wheel_odom_node = sensor_integration.wheel_odom_node:main',
        ],
    },
)
