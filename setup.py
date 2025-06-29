import os
from glob import glob

from setuptools import find_packages, setup

package_name = "sign_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.xml")),
        (os.path.join("share", package_name, "rviz2"), glob("rviz2/*")),
        (os.path.join("share", package_name, "template_pcd"), glob("template_pcd/*")),
        (
            os.path.join("share", package_name, "hyperparameter"),
            glob("hyperparameter/pcd_bag/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yutarop",
    maintainer_email="yutarop.storm.7@gmail.com",
    description="ROS2 package for detecting road-closed signs, designed for Task C of the Tsukuba Challenge.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sign_detection = sign_detection.sign_detection:main",
        ],
    },
)
