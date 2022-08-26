"""Launch a controller_server node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create a controller_server node with a launch description."""
    ezrassor_arm_moveit_ros2_node = Node(
        package="ezrassor_arm_moveit_ros2",
        # executable="controller_server",
        # output={"both": "screen"},
    )

    return LaunchDescription([ezrassor_arm_moveit_ros2_node])
