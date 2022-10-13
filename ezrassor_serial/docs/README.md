ezrassor_sim_description
---------------------
![Build Badge](https://github.com/FlaSpaceInst/ezrassor_sim_description/workflows/Build/badge.svg) 
![Style Badge](https://img.shields.io/badge/Code%20Style-black-000000.svg)

The `ezrassor_sim_description` package contains the xacro description for the ezrassor robot and spawns a new robot into a running Gazebo simulation.

usage
-----
```
command:
  ros2 launch ezrassor_sim_description spawn_ezrassor.py [argument:=value]

optional arguments:
  x             [lowercase] float value representing the spawn value for the robot along the x axis in Gazebo (default 0.0)
  y             [lowercase] float value representing the spawn value for the robot along the y axis in Gazebo (default 0.0)
  z             [lowercase] float value representing the spawn value for the robot along the z axis in Gazebo (default 0.0)
  R             [UPPERCASE] float value representing the spawn roll degree for the robot in Gazebo (default 0.0)
  P             [UPPERCASE] float value representing the spawn pitch degree for the robot in Gazebo (default 0.0)
  Y             [UPPERCASE] float value representing the spawn yaw degree for the robot in Gazebo (default 0.0)
  robot_name    string value for the unique name of the robot, used for topic name and namespace (default ezrassor)
  model         string value for the full path of the robot xacro file (default ezrassor_sim_description/urdf/ezrassor.xacro)
```

example
--------
Launch the Gazebo client with the name `ezrassor_1`:
``` sh
# First launch Gazebo and then spawn the robot with the controller manager
ros2 launch ezrassor_sim_gazebo gazebo_launch.py
ros2 launch ezrassor_sim_description spawn_ezrassor.py robot_name:=ezrassor_1
```   

testing
-------
Changes to this package will run through the linters specified in the GitHub Actions workflow which include:
- Black formatting check
- PEP8 compliance check

These tests will run automatically when changes are checked in to the `development` branch.  

Before you check in changes, please test your changes locally:

```sh
python3 -m pip install --upgrade black flake8
python3 -m black --check .
python3 -m flake8 .
```
