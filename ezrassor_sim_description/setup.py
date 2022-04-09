"""Setup the ezrassor_sim_description module.
"""
from setuptools import setup
import glob


setup(
    name="ezrassor_sim_description",
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
    packages=["arms_driver", "wheels_driver", "drums_driver", "paver_arm_driver"],
    package_dir={"": "source"},
    install_requires=["setuptools"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resources/ezrassor_sim_description"],
        ),
        (
            "share/ezrassor_sim_description",
            ["package.xml"] + glob.glob("launch/*"),
        ),
        ("share/ezrassor_sim_description/meshes", glob.glob("meshes/*")),
        ("share/ezrassor_sim_description/urdf", glob.glob("urdf/*")),
        ("share/ezrassor_sim_description/config", glob.glob("config/*")),
    ],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arms_driver = arms_driver.__main__:main",
            "wheels_driver = wheels_driver.__main__:main",
            "drums_driver = drums_driver.__main__:main",
            "paver_arm_driver = parver_arm_driver.__main__:main",
        ],
    },
)
