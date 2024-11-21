from setuptools import setup
import os
from glob import glob

package_name = "operasim_nav2"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
        (
            os.path.join("share", package_name, "behavior_trees"),
            glob(os.path.join("behavior_trees", "*.xml")),
        ),
        (
            os.path.join("share", package_name, "maps"),
            glob(os.path.join("maps", "*.pgm")),
        ),
        (
            os.path.join("share", package_name, "maps"),
            glob(os.path.join("maps", "*.yaml")),
        ),
        
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wangyongdong",
    maintainer_email="wangyongdong@robot.t.u-tokyo.ac.jp",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_pose_to_tf2_broadcaster = operasim_nav2.odom_pose_to_tf2_broadcaster:main",
            "adapter_tf2_broadcaster_4_toy_robots = operasim_nav2.adapter_tf2_broadcaster_4_toy_robots:main"
        ],
    },
)
