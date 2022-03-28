from setuptools import setup

package_name = 'autonomous_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nate',
    maintainer_email='wilk.nathan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detection = autonomous_control.obstacle_detection:main',
            'autonomous_control = autonomous_control.autonomous_control:on_start_up',
            'interactor = autonomous_control.simulation_interact:main'
        ],
    },
)
