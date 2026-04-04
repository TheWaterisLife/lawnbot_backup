from setuptools import find_packages, setup

package_name = 'lawnbot_comms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lawnbot',
    maintainer_email='lawnbot@todo.todo',
    description='Lawnbot comms bridge (WebSocket <-> ROS 2)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = lawnbot_comms.bridge_node:main',
            'fake_telemetry_node = lawnbot_comms.fake_telemetry_node:main',
        ],
    },
)
