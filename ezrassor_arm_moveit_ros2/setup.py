"""Setup the ezrassor_arm_moveit_ros2 module.
"""
from setuptools import setup
import glob
import os.path
import os


def get_data_files():
    """Manually specify data_files list."""
    files = [
        (
            "share/ament_index/resource_index/packages",
            ["resources/ezrassor_arm_moveit_ros2"],
        ),
        ("share/ezrassor_arm_moveit_ros2", ["package.xml"]),
        ("share/ezrassor_arm_moveit_ros2/launch", glob.glob("launch/*")),
        ("share/ezrassor_arm_moveit_ros2/config", glob.glob("config/*")), 
        ("share/ezrassor_arm_moveit_ros2/urdf", glob.glob("urdf/*")),
    ]

    return files


setup(
    name="ezrassor_arm_moveit_ros2",
    version="2.0.0",
    description="Start a Gazebo world for the EZRASSOR to spawn in.",
    maintainer="EZRASSOR Team",
    maintainer_email="ez.rassor@gmail.com",
    license="MIT",
    keywords=["EZRASSOR", "ROS", "ISRU", "NASA", "Rover", "UCF", "Robotics"],
    classifiers=[
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python",
        "Topic :: Education",
        "Topic :: Scientific/Engineering :: Astronomy",
        "Topic :: Scientific/Engineering :: Physics",
    ],
    packages=["ezrassor_arm_moveit_ros2"],
    package_dir={"": "source"},
    install_requires=["setuptools"],
    data_files=get_data_files(),
)
