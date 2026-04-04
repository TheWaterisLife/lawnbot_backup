from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mower_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lawnbot',
    maintainer_email='nicolasgharzouzi04@gmail.com',
    description='Mower autonomy stack',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_node = mower_autonomy.localization_node:main',
            'encoder_node = mower_autonomy.encoder_node:main',
            'autonomy_node = mower_autonomy.autonomy_node:main',
            'imu_node = mower_autonomy.imu_node:main',
            # if you created the websocket start bridge earlier, include it:
            # 'start_ws_bridge = mower_autonomy.start_ws_bridge:main',
        ],
    },
)
