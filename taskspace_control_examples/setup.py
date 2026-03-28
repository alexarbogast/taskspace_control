import os
from setuptools import find_packages, setup
from glob import glob

package_name = "taskspace_control_examples"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "numpy-quaternion",
    ],
    zip_safe=True,
    maintainer="Alex Arbogast",
    maintainer_email="arbogastaw@gmail.com",
    description="ros2_control examples using taskspace controllers",
    license="Apache License, Version 2.0",
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "pose_control_demo = \
                    taskspace_control_examples.pose_control_demo:main",
        ],
    },
)
