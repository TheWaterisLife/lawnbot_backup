from setuptools import find_packages, setup


package_name = "ai_camera_vision"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="capstone camera",
    maintainer_email="you@example.com",
    description="ROS2 sensor node that publishes OAK-D Lite dual-model inference outputs (detections + segmentation).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ai_camera_vision_node = ai_camera_vision.node:main",
        ],
    },
)


