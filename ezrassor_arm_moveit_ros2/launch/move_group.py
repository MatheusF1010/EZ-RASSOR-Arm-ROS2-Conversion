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


def generate_launch_description():
    # GDB Debug Option
        # Not important right now

    # Verbose Mode Option right now
        # Not important

    # move_group settings
    pipeline_launch_arg = DeclareLaunchArgument(
            name='pipeline',
            default_value='ompl'
    )
    allow_trajectory_execution_launch_arg = DeclareLaunchArgument(
            name='allow_trajectory_execution',
            default_value='true'
    )
    fake_execution_launch_arg = DeclareLaunchArgument(
            name='fake_execution',
            default_value='false'
    )
    execution_type_launch_arg = DeclareLaunchArgument(
        name='execution_type',
        default_value='interpolate'
    )
    max_safe_path_cost_launch_arg = DeclareLaunchArgument(
        name='max_safe_path_cost',
        default_value='1'
    )
    jiggle_fraction_launch_arg = DeclareLaunchArgument(
        name='jiggle_fraction',
        default_value='0.05'
    )
    publish_monitored_planning_scene_launch_arg = DeclareLaunchArgument(
        name='publish_monitored_planning_scene',
        default_value='true'
    )
    capabilities_launch_arg = DeclareLaunchArgument(
        name='capabilities',
        default_value=''
    )
    disable_capabilities_launch_arg = DeclareLaunchArgument(
        name='disable_capabilities',
        default_value=''
    )
    load_robot_description_launch_arg = DeclareLaunchArgument(
        name='load_robot_description',
        default_value='true'
    )

    # load URDF, SRDF and joint_limits configuration
        # planning_context.launch should go here
        # Load /urdf/arm_model.urdf 
        # Load config/ezrassor.srdf
    
    # joint_limits.yaml
    joint_limits = { 

        'grabber_joint1': {
            'has_velocity_limits': 'true',
            'max_velocity': 1,
            'has_acceleration_limits': 'false',
            'max_acceleration': 0
        },
        'grabber_joint2': {
            'has_velocity_limits': 'true',
            'max_velocity': 1,
            'has_acceleration_limits': 'false',
            'max_acceleration': 0
        },
        'joint12': {
            'has_velocity_limits': 'true',
            'max_velocity': 1,
            'has_acceleration_limits': 'false',
            'max_acceleration': 1
        },
        'joint23': {
            'has_velocity_limits': 'true',
            'max_velocity': 1,
            'has_acceleration_limits': 'false',
            'max_acceleration': 1
        },
        'joint34': {
            'has_velocity_limits': 'true',
            'max_velocity': 1,
            'has_acceleration_limits': 'false',
            'max_acceleration': 1
        },
        'joint45': {
            'has_velocity_limits': 'true',
            'max_velocity': 1,
            'has_acceleration_limits': 'false',
            'max_acceleration': 1
        },
        'joint56': {
            'has_velocity_limits': 'true',
            'max_velocity': 1,
            'has_acceleration_limits': 'false',
            'max_acceleration': 1
        }
    }

    # cartesian_limits.yaml
    cartesian_limits = { 
        'max_trans_vel': 1,
        'max_trans_acc': 2.25,
        'max_trans_dec': -5,
        'max_rot_vel': 1.57
    }

    # kinematics.yaml
    kinematics = { 

        'moveit_arm_controller': { # Might have to change  **********************************************************
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution': 0.005,
            'kinematics_solver_timeout': 0.005
        },
        'gripper_controller': {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution': 0.005,
            'kinematics_solver_timeout': 0.005
        }
    }


    # Planning Functionality
    planning_pipeline(LaunchConfiguration("pipeline"), LaunchConfiguration("capabilities"), LaunchConfiguration("disable_capabilities"))



    # Trajectory Execution Functionality


    # Sensors Functionality


    # Start the actual move_group node/action server
        # Set the display variable, in case OpenGL code is used internally
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot 


    return LaunchDescription([
        pipeline_launch_arg,
        allow_trajectory_execution_launch_arg,
        fake_execution_launch_arg,
        execution_type_launch_arg,
        max_safe_path_cost_launch_arg,
        jiggle_fraction_launch_arg,
        publish_monitored_planning_scene_launch_arg,
        capabilities_launch_arg,
        disable_capabilities_launch_arg,
        load_robot_description_launch_arg
    ])


# def planning_context(load_robot_description=LaunchConfiguration("load_robot_description")):

def planning_pipeline(pipeline="ompl", capabilities="", disable_capabilities=""):
    ompl_planning_pipeline(capabilities, disable_capabilities)
    

def ompl_planning_pipeline(capabilities="", disable_capabilities=""):

    planning_plugin = "ompl_interface/OMPLPlanner"

    planning_adapters = "default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints"

    start_state_max_bounds_error = 0.1

    # PARAMS
    planning_plugin = "ompl_interface/OMPLPlanner"

    request_adapters = "default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints"

    start_state_max_bounds_error = 0.1

    capabilities = ""

    disable_capabilities = ""
    
    # Load ompl_planning.yaml with python. Replace other variables (lines 71-126) *****************************************************************


if __name__ == '__main__':
    generate_launch_description()