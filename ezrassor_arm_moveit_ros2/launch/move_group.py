from ntpath import join
import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    RegisterEventHandler)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
from launch.substitutions.find_executable import FindExecutable
import yaml


def load_yaml_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
            #return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None



def generate_launch_description():
        
    # execution_type = "interpolate" #ros controller.yaml
    max_safe_path_cost = 1
    jiggle_fraction = 0.05
    
    #ompl.yaml
    planning_plugin = "ompl_interface/OMPLPlanner" 
    planning_adapters = "default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints"
    start_state_max_bounds_error = 0.1
    
    pkg_ezrassor_arm_moveit_ros2 = os.path.join(
        get_package_share_directory("ezrassor_arm_moveit_ros2")
    )

    # loading arm_model.urdf
    urdf_file = os.path.join(
        pkg_ezrassor_arm_moveit_ros2, "urdf", "arm_model.urdf"
    )
    robot_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file,
            " ",
        ]
    )

    controller = load_yaml_file(pkg_ezrassor_arm_moveit_ros2, "config/ros_controllers.yaml")
    ompl_planning = load_yaml_file(pkg_ezrassor_arm_moveit_ros2, "config/ompl_planning.yaml")
    joint_limits = load_yaml_file(pkg_ezrassor_arm_moveit_ros2, "config/joint_limits.yaml")
    cartesian_limits = load_yaml_file(pkg_ezrassor_arm_moveit_ros2, "config/cartesian_limits.yaml")
    kinematics = load_yaml_file(pkg_ezrassor_arm_moveit_ros2, "config/kinematics.yaml")
    sensors_3d = load_yaml_file(pkg_ezrassor_arm_moveit_ros2, "config/sensors_3d.yaml")
    robot_desc = load_yaml_file(pkg_ezrassor_arm_moveit_ros2, "config/ezrassor.srdf")

    param = {
        "robot_description": robot_urdf
        }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[param]
    )
    
    trajectory_execution_allowed_execution_duration_scaling = 1.2
    trajectory_execution_allowed_goal_duration_margin = 0.5
    trajectory_execution_allowed_start_tolerance = 0.01
    moveit_controller_manager = "moveit_simple_controller_manager/MoveItSimpleControllerManager"


    # Sensors Functionality
    # moveit_sensor_manager="ezrassor"
    octomap_resolution = 0.025
    max_range = 5.0
    # LOAD config/sensors_3d.yaml ********************************************************


    # Start the actual move_group node/action server
        # Set the display variable, in case OpenGL code is used internally
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot 

    move_group = Node(
        name="move_group",
        namespace='/ezrassor',
        package='moveit_ros_move_group',
        executable='move_group',
        respawn="false",
        output="screen",
        
        # ENV *****************************************
        
        parameters=[{
            "allow_trajectory_execution": True,
            "max_safe_path_cost": max_safe_path_cost,
            "jiggle_fraction": jiggle_fraction,
            "planning_scene_monitor/publish_planning_scene": True,
            "planning_scene_monitor/publish_geometry_updates": True,
            "planning_scene_monitor/publish_state_updates": True,
            "planning_scene_monitor/publish_transforms_updates": True,
            "robot_description": robot_urdf,
            "moveit_simple_controller_manager": controller,
            "ompl_planning": ompl_planning,
            "robot_description_planning": joint_limits,
            "robot_description_planning": cartesian_limits,
            "robot_description_kinematics": kinematics,
            "robot_description_semantic": robot_desc,
            "octomap_resolution": octomap_resolution,
            "max_range": max_range,
            "moveit_controller_manager": moveit_controller_manager,
            "moveit_manage_controllers": True,
            "trajectory_execution/allowed_execution_duration_scaling": trajectory_execution_allowed_execution_duration_scaling,
            "trajectory_execution/allowed_goal_duration_margin": trajectory_execution_allowed_goal_duration_margin,
            "trajectory_execution/allowed_start_tolerance": trajectory_execution_allowed_start_tolerance,
            "planning_plugin": planning_plugin,
            "request_adapters": planning_adapters,
            "start_state_max_bounds_error": start_state_max_bounds_error,
        }]
        
    )

    tf2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["0", "0", "0", "1", "0", "0", "Authority undetectable", "grabber1"],
        parameters=[{
            "robot_description": robot_urdf
        }]
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "grabber1"],
        parameters=[{
            "robot_description": robot_urdf
        }]
    )


    return LaunchDescription([
        # robot_state_publisher,
        move_group,
        # tf2,
    ])

if __name__ == '__main__':
    generate_launch_description()





# loading ros_controllers.yaml
    # controller_yaml_file = os.path.join(
    #     pkg_ezrassor_arm_moveit_ros2, "config", "ros_controllers.yaml"
    # )
    # with open(controller_yaml_file, 'r') as file:
    #     controller = yaml.safe_load(file)

    # loading ompl_planning.yaml
    # ompl_planning_yaml_file = os.path.join(
    #     pkg_ezrassor_arm_moveit_ros2, "config", "ompl_planning.yaml"
    # )
    # with open(ompl_planning_yaml_file, 'r') as file:
    #     ompl_planning = yaml.safe_load(file)

    # loading joint_limits.yaml
    # joint_limits_yaml_file = os.path.join(
    #     pkg_ezrassor_arm_moveit_ros2, "config", "joint_limits.yaml"
    # )
    # with open(joint_limits_yaml_file, 'r') as file:
    #     joint_limits = yaml.safe_load(file)

    # loading cartesian_limits.yaml
    # cartesian_limits_yaml_file = os.path.join(
    #     pkg_ezrassor_arm_moveit_ros2, "config", "cartesian_limits.yaml"
    # )
    # with open(cartesian_limits_yaml_file, 'r') as file:
    #     cartesian_limits = yaml.safe_load(file)

    # loading kinematics.yaml
    # kinematics_yaml_file = os.path.join(
    #     pkg_ezrassor_arm_moveit_ros2, "config", "kinematics.yaml"
    # )
    # with open(kinematics_yaml_file, 'r') as file:
    #     kinematics = yaml.safe_load(file)

    # loading sensors_3d.yaml
    # sensors_3d_yaml_file = os.path.join(
    #     pkg_ezrassor_arm_moveit_ros2, "config", "sensors_3d.yaml"
    # )
    # with open(sensors_3d_yaml_file, 'r') as file:
    #     sensors_3d = yaml.safe_load(file)

    # loading ezrassor.srdf
    # ezrassor_srdf_file = os.path.join(
    #     pkg_ezrassor_arm_moveit_ros2, "config", "ezrassor.srdf"
    # )
    # with open(ezrassor_srdf_file, 'r') as infp:
    #     robot_desc = infp.read()