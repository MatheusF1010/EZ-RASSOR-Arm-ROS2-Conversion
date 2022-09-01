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


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    debug_launch_arg = DeclareLaunchArgument(
        "debug", default_value=TextSubstitution(text="false")
    )

    if LaunchConfiguration("debug") == "true":
        launch_prefix_launch_arg = DeclareLaunchArgument(
            "launch_prefix",
            default_value='gdb -x $(find-pkg-share ezrassor_arm_moveit_ros2)/launch/gdb_settings.gdb --ex run --args'
        )
    else:
        launch_prefix_launch_arg = DeclareLaunchArgument(
            "launch_prefix",
            default_value=''
        )
    
    info_launch_arg = DeclareLaunchArgument(
        "info", default_value=LaunchConfiguration("debug")
    )

    if LaunchConfiguration("info") == "true":
        command_args_launch_arg = DeclareLaunchArgument(
            "command_args",
            default_value='--debug'
        )
    else:
        command_args_launch_arg = DeclareLaunchArgument(
            "command_args",
            default_value=''
        )

    pipeline_launch_arg = DeclareLaunchArgument(
        "pipeline", default_value=TextSubstitution(text="ompl")
    )

    allow_trajectory_execution_launch_arg = DeclareLaunchArgument(
        "allow_trajectory_execution", default_value=TextSubstitution(text="true")
    )

    fake_execution_launch_arg = DeclareLaunchArgument(
        "fake_execution", default_value=TextSubstitution(text="false")
    )

    execution_type_launch_arg = DeclareLaunchArgument(
        "execution_type", default_value=TextSubstitution(text="interpolate")
    )

    max_safe_path_cost_launch_arg = DeclareLaunchArgument(
        "max_safe_path_cost", default_value=TextSubstitution(text="1")
    )

    jiggle_fraction_launch_arg = DeclareLaunchArgument(
        "jiggle_fraction", default_value=TextSubstitution(text="0.05")
    )

    publish_monitored_planning_scene_launch_arg = DeclareLaunchArgument(
        "publish_monitored_planning_scene", default_value=TextSubstitution(text="true")
    )

    capabilities_launch_arg = DeclareLaunchArgument(
        "capabilities", default_value=TextSubstitution(text="")
    )

    disable_capabilities_launch_arg = DeclareLaunchArgument(
        "disable_capabilities", default_value=TextSubstitution(text="")
    )

    load_robot_description_launch_arg = DeclareLaunchArgument(
        "load_robot_description", default_value=TextSubstitution(text="true")
    )

    # include another launch file
    # planning_context_launch_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #         get_package_share_directory('ezrassor_arm_moveit_ros2'),
    #         'launch/planning_context.launch'),
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

    # if LaunchConfiguration("allow_trajectory_execution") == "true":
    #     if LaunchConfiguration("fake_execution") == "true":
    #         trajectory_execution_launch_include = GroupAction(
    #             actions=[
    #                 # push-ros-namespace to set namespace of included nodes
    #                 PushRosNamespace('move_group'),
    #                 IncludeLaunchDescription(
    #                     PythonLaunchDescriptionSource(
    #                         os.path.join(
    #                             get_package_share_directory('ezrassor_arm_moveit_ros2'),
    #                             'launch/sensor_manager.launch.xml')
    #                     ),
    #                     launch_arguments=[{
    #                         "moveit_manage_controllers": LaunchConfiguration("true"),
    #                         "moveit_controller_manager": LaunchConfiguration('fake'),
    #                         "execution_type": LaunchConfiguration('execution_type')
    #                     }]
    #                 )
    #             ]
    #         )
    #     else:
    #         trajectory_execution_launch_include = GroupAction(
    #             actions=[
    #                 # push-ros-namespace to set namespace of included nodes
    #                 PushRosNamespace('move_group'),
    #                 IncludeLaunchDescription(
    #                     PythonLaunchDescriptionSource(
    #                         os.path.join(
    #                             get_package_share_directory('ezrassor_arm_moveit_ros2'),
    #                             'launch/sensor_manager.launch.xml')
    #                     ),
    #                     launch_arguments=[{
    #                         "moveit_manage_controllers": LaunchConfiguration("true"),
    #                         "moveit_controller_manager": LaunchConfiguration('ezrassor'),
    #                         "execution_type": LaunchConfiguration('execution_type')
    #                     }]
    #                 )
    #             ]
    #         )

    # if LaunchConfiguration("allow_trajectory_execution") == "true":
    #     sensor_manager_launch_include = GroupAction(
    #         actions=[
    #             # push-ros-namespace to set namespace of included nodes
    #             PushRosNamespace('move_group'),
    #             IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource(
    #                     os.path.join(
    #                         get_package_share_directory('ezrassor_arm_moveit_ros2'),
    #                         'launch/sensor_manager.launch.xml')
    #                 ),
    #                 launch_arguments=[{
    #                     "moveit_sensor_manager": LaunchConfiguration("ezrassor"),
    #                 }]
    #             )
    #         ]
    #     )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        # launch_prefix_launch_arg=LaunchConfiguration("launch_prefix"),
        respawn='false',
        output='screen',
        arguments=[{
            LaunchConfiguration("command_args"),
            LaunchConfiguration("launch_prefix")
        }],
        #env=
        parameters=[{
            "allow_trajectory_execution": LaunchConfiguration('allow_trajectory_execution'),
            "max_safe_path_cost": LaunchConfiguration('max_safe_path_cost'),
            "jiggle_fraction": LaunchConfiguration('jiggle_fraction'),
            "planning_scene_monitor/publish_planning_scene": LaunchConfiguration('publish_monitored_planning_scene'),
            "planning_scene_monitor/publish_geometry_updates": LaunchConfiguration('publish_monitored_planning_scene'),
            "planning_scene_monitor/publish_state_updates": LaunchConfiguration('publish_monitored_planning_scene'),
            "planning_scene_monitor/publish_transforms_updates": LaunchConfiguration('publish_monitored_planning_scene'),
        }]
    )    

    return LaunchDescription([
        debug_launch_arg,
        launch_prefix_launch_arg,
        info_launch_arg,
        command_args_launch_arg,
        pipeline_launch_arg,
        allow_trajectory_execution_launch_arg,
        fake_execution_launch_arg,
        execution_type_launch_arg,
        max_safe_path_cost_launch_arg,
        jiggle_fraction_launch_arg,
        publish_monitored_planning_scene_launch_arg,
        capabilities_launch_arg,
        disable_capabilities_launch_arg,
        load_robot_description_launch_arg,
        # planning_context_launch_include,
        # planning_pipeline_launch_include,
        # trajectory_execution_launch_include,
        # sensor_manager_launch_include,
        move_group_node,
    ])