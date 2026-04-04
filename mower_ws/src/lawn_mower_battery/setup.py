from setuptools import setup
import os
from glob import glob

package_name = 'lawn_mower_battery'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
    description='Battery monitoring node for autonomous lawn mower',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_monitor = lawn_mower_battery.battery_monitor:main',
            'status_led_node = lawn_mower_battery.status_led_node:main',
        ],
    },
)
