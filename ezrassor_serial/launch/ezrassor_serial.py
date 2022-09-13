"""Launch Gazebo with the specified world file (empty world by default)"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions.opaque_function import OpaqueFunction
from launch.substitutions.find_executable import FindExecutable
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
import os



def generate_launch_description():
    """Spawn a new instance of Gazebo Classic with an optional world file."""

    paver_arm_manual_serial_driver_node = Node(
        package="ezrassor_serial",
        executable="paver_arm_manual_serial_driver",
        namespace="/ezrassor",
        output={"both": "screen"},
    )

    # Note that this package WILL NOT start Gazebo
    # instead, when this launch file is executed it will wait for /spawn_entity
    # to be available. This will automatically be available after Gazebo is launched
    # from ROS2.
    return LaunchDescription(
        [
            paver_arm_manual_serial_driver_node
        ]
    )
