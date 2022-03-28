# EZRASSOR-Arm-ROS2-Conversion

Once turtlesim is completed, workspace will be:

```
/dev_ws/
|_> /build/
|_> /install/
|_> /log/
|_> /src/
```

Files in this repo are all packages and will go in the /src/ directory.

To run packages:

1) Source ROS2 environment
```
source /opt/ros/foxy/setup.bash
```

2) Source Gazebo environment
```
. /usr/share/gazebo/setup.sh
```

3)  Resolve dependencies
```
cd /dev_ws/
rosdep install -i --from-path src --rosdistro foxy -y
```

3) Build workspace
```
colcon build
```

4) Source overlay
```
. install/local_setup.bash
```

5) Run ROS2 script. Repeat 1-5 in seperate terminals.

-------------------------------------------------------

Example: To launch a rover on Gazebo run:

TERMINAL 1:
```
source /opt/ros/foxy/setup.bash
. /usr/share/gazebo/setup.sh
cd /dev_ws/
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
. install/local_setup.bash
ros2 launch ezrassor_sim_gazebo gazebo_launch.py
```

TERMINAL 2:
```
source /opt/ros/foxy/setup.bash
. /usr/share/gazebo/setup.sh
cd /dev_ws/
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
. install/local_setup.bash
ros2 launch ezrassor_sim_description spawn_ezrassor.py
```
