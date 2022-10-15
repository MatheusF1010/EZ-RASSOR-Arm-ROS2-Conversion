"""Setup the ezrassor_serial module.
"""
from setuptools import setup
import glob


setup(
    name="ezrassor_serial",
    version="2.0.0",
    description="Build the EZRASSOR simulation model from urdf \
        and spawn it into Gazebo",
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
    packages=["paver_arm_serial_driver"],
    package_dir={"": "source"},
    install_requires=["setuptools"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resources/ezrassor_serial"],
        ),
        (
            "share/ezrassor_serial",
            ["package.xml"] + glob.glob("launch/*"),
        ),
    ],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "paver_arm_serial_driver = paver_arm_serial_driver.__main__:main",
        ],
    },
)
