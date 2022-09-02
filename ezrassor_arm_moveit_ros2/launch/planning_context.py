from distutils.text_file import TextFile
from multiprocessing import Value
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import SetEnvironmentVariable


should_launch_load_robot_description = LaunchConfiguration('load_robot_description')

def generate_launch_description():

    load_robot_description_arg = DeclareLaunchArgument(
        "load_robot_description", default_value=TextSubstitution(text="false")
    )

    robot_description_launch_arg = DeclareLaunchArgument(
        "robot_description", default_value=TextSubstitution(text="robot_description")
    )

    # if IfCondition(should_launch_load_robot_description): # If true, create this argument
    #     var_robot_description_launch_arg = DeclareLaunchArgument(
    #         "name", 
    #         TextFile(
    #             PythonLaunchDescriptionSource(
    #             os.path.join(
    #                 get_package_share_directory('ezrassor_arm_moveit_ros2'),
    #                 'launch/planning_context.py')
    #             )
    #         )
    #     )

    # parameters=[{
    #         "allow_trajectory_execution": LaunchConfiguration('allow_trajectory_execution'),
    #         "max_safe_path_cost": LaunchConfiguration('max_safe_path_cost'),
    #         "jiggle_fraction": LaunchConfiguration('jiggle_fraction'),
    #         "planning_scene_monitor/publish_planning_scene": LaunchConfiguration('publish_monitored_planning_scene'),
    #         "planning_scene_monitor/publish_geometry_updates": LaunchConfiguration('publish_monitored_planning_scene'),
    #         "planning_scene_monitor/publish_state_updates": LaunchConfiguration('publish_monitored_planning_scene'),
    #         "planning_scene_monitor/publish_transforms_updates": LaunchConfiguration('publish_monitored_planning_scene'),
    #     }]

    # planning_context_launch_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #         get_package_share_directory('ezrassor_arm_moveit_ros2'),
    #         'launch/planning_context.py'),
    #     ),
    #     launch_arguments=[{
    #         "load_robot_description": LaunchConfiguration("load_robot_description")
    #     }]
    # )

    # planning_pipeline_launch_include = GroupAction(
    #     actions=[
    #         # push-ros-namespace to set namespace of included nodes
    #         PushRosNamespace('move_group'),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(
    #                     get_package_share_directory('ezrassor_arm_moveit_ros2'),
    #                     'launch/planning_pipeline.launch.xml')
    #             ),
    #             launch_arguments=[{
    #                 "pipeline": LaunchConfiguration("pipeline"),
    #                 "capabilities": LaunchConfiguration('capabilities'),
    #                 "disable_capabilities": LaunchConfiguration('disable_capabilities')
    #             }],

    #             # Unexpected argument
    #             # parameters=[{
    #             # "capabilities": LaunchConfiguration('capabilities'),
    #             # "disable_capabilities": LaunchConfiguration('disable_capabilities'),
    #             # }],
                
    #         )
    #     ]
    # )

    # Unless tag launch file contents.
    # unless_trajectory_execution_launch_include = GroupAction(
    #     condition=IfCondition(should_launch_allow_trajectory_execution), # If condition is true, proceed to include?
    #     actions=[
    #         PushRosNamespace('move_group'),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(
    #                     get_package_share_directory('ezrassor_arm_moveit_ros2'),
    #                     'launch/trajectory_execution.launch.xml')
    #             ),
    #             condition=UnlessCondition(should_launch_moveit_controller_manager), # Unless condition is true, launch args?
    #             launch_arguments=[{
    #                 "moveit_manage_controllers": LaunchConfiguration("true"),
    #                 "moveit_controller_manager": LaunchConfiguration('ezrassor'),
    #                 "execution_type": LaunchConfiguration('execution_type')
    #             }]
    #         )
    #     ]
    # )
    
    # ONLY INCLUDES THAT DOES NOT THROW ERROR BECAUSE IF CONDITION IS FALSE. 
    # Therefore it will not launch this includes action. Will only launch previous one. One or the other.

    # If tag launch file contents.
    # if_trajectory_execution_launch_include = GroupAction(
    #     condition=IfCondition(should_launch_allow_trajectory_execution), # If condition is true, procees to include?
    #     actions=[
    #         PushRosNamespace('move_group'),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(
    #                     get_package_share_directory('ezrassor_arm_moveit_ros2'),
    #                     'launch/trajectory_execution.launch.xml')
    #             ),
    #             condition=IfCondition(should_launch_moveit_controller_manager), # If condition is true, launch args? (means previous Unless was true, does not launch previous node?)
    #             launch_arguments=[{
    #                 "moveit_manage_controllers": LaunchConfiguration("true"),
    #                 "moveit_controller_manager": LaunchConfiguration('fake'), 
    #                 "execution_type": LaunchConfiguration('execution_type')
    #             }]
    #         )
    #     ]
    # )

    # sensor_manager_launch_include = GroupAction(
    #     condition=IfCondition(should_launch_allow_trajectory_execution), # If condition is true, procees to include?
    #     actions=[
    #         # push-ros-namespace to set namespace of included nodes
    #         PushRosNamespace('move_group'),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(
    #                     get_package_share_directory('ezrassor_arm_moveit_ros2'),
    #                     'launch/sensor_manager.launch.xml')
    #             ),
    #             launch_arguments=[{
    #                 "moveit_sensor_manager": LaunchConfiguration("ezrassor"),
    #             }]
    #         )
    #     ]
    # )    

    return LaunchDescription([
        load_robot_description_arg,
        robot_description_launch_arg,
        # var_robot_description_launch_arg
    ])