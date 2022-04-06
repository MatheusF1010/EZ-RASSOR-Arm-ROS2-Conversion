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


def __spawn_robot(context, *args, **kwargs):
    """Returns the nodes for spawning a unique robot in Gazebo."""

    pkg_ezrassor_sim_description = os.path.join(
        get_package_share_directory("ezrassor_sim_description")
    )

    # Make sure that the robot name does not have a leading /
    # (this variable is what the spawn_entity service will use as a unique name)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    if robot_name[0] == "/":
        robot_name = robot_name[1:]

    xacro_file = os.path.join(
        pkg_ezrassor_sim_description, "urdf", "ezrassor.xacro.urdf"
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "name:=",
            robot_name,
        ]
    )
    params = {"robot_description": robot_description_content}

    # Since we are building the namespace off of the robot name, ensure
    # it has a leading /
    namespace = robot_name
    if namespace[0] != "/":
        namespace = f"/{namespace}"

    # State publisher needs to be tied to the unique instance of the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[params],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        namespace=namespace,
        arguments=[
            "-entity",
            robot_name,
            "-robot_namespace",
            namespace,
            "-topic",
            f"{namespace}/robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-R",
            LaunchConfiguration("R"),
            "-P",
            LaunchConfiguration("P"),
            "-Y",
            LaunchConfiguration("Y"),
        ],
        output="screen",
    )

    # controllers must be loaded here as part of the spawn or else the
    # gazebo_ros controls won't know what to make controllers for
    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "-c",
            f"/{robot_name}/controller_manager",
            "--set-state",
            "start",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "-c",
            f"/{robot_name}/controller_manager",
            "--set-state",
            "start",
            "diff_drive_controller",
        ],
        output="screen",
    )

    load_arm_front_velocity_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "-c",
            f"/{robot_name}/controller_manager",
            "--set-state",
            "start",
            "arm_front_velocity_controller",
        ],
        output="screen",
    )

    load_arm_back_velocity_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "-c",
            f"/{robot_name}/controller_manager",
            "--set-state",
            "start",
            "arm_back_velocity_controller",
        ],
        output="screen",
    )

    load_drum_front_velocity_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "-c",
            f"/{robot_name}/controller_manager",
            "--set-state",
            "start",
            "drum_front_velocity_controller",
        ],
        output="screen",
    )

    load_drum_back_velocity_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "-c",
            f"/{robot_name}/controller_manager",
            "--set-state",
            "start",
            "drum_back_velocity_controller",
        ],
        output="screen",
    )

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_front_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_back_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_drum_front_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_drum_back_velocity_controller],
            )
        ),
        robot_state_publisher,
        spawn_entity,
    ]


def generate_launch_description():
    """Spawn a new instance of Gazebo Classic with an optional world file."""

    robot_name_argument = DeclareLaunchArgument(
        "robot_name",
        default_value="ezrassor",
        description="Entity name and namespace for robot spawn (default: ezrassor)",
    )

    # spawn_entity.py takes in these arguments separately,
    # doing the same here for consistency.
    x_position_argument = DeclareLaunchArgument(
        "x",
        default_value="0.0",
        description="X position for robot spawn: [float]",
    )
    y_position_argument = DeclareLaunchArgument(
        "y",
        default_value="0.0",
        description="Y position for robot spawn: [float]",
    )
    z_position_argument = DeclareLaunchArgument(
        "z",
        default_value="0.2",
        description="Z position for robot spawn: [float]",
    )
    r_axis_argument = DeclareLaunchArgument(
        "R",
        default_value="0.0",
        description="Roll angle for robot spawn: [float]",
    )
    p_axis_argument = DeclareLaunchArgument(
        "P",
        default_value="0.0",
        description="Pitch angle for robot spawn: [float]",
    )
    y_axis_argument = DeclareLaunchArgument(
        "Y",
        default_value="0.0",
        description="Yaw angle for robot spawn: [float]",
    )

    arms_driver_node = Node(
        package="ezrassor_sim_description",
        executable="arms_driver",
        namespace=LaunchConfiguration("robot_name"),
        output={"both": "screen"},
    )
    wheels_driver_node = Node(
        package="ezrassor_sim_description",
        executable="wheels_driver",
        namespace=LaunchConfiguration("robot_name"),
        output={"both": "screen"},
    )
    drums_driver_node = Node(
        package="ezrassor_sim_description",
        executable="drums_driver",
        namespace=LaunchConfiguration("robot_name"),
        output={"both": "screen"},
    )

    #param_config = os.path.join(get_package_share_directory('depthimage_to_laserscan'), 'cfg', 'param.yaml')
    depth_img_to_ls = Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='depthimage_to_laserscan_node',
            remappings=[('depth','/ezrassor/depth_camera/depth/image_raw'),
                        ('depth_camera_info', '/ezrassor/depth_camera/depth/camera_info'),
                        ('scan', '/obstacle_detection/combined')],
            parameters=[{
                    "output_frame": "camera_link",
                    "scan_time": 0.033,
                    "range_min": 0.105,
                    "range_max": 10.0,
                    "scan_height": 1
            }]
    )


    # pc_to_ls = Node(
    #         package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    #         remappings=[('cloud_in', ['ezrassor/depth_camera/points']),
    #                     ('scan', ['/obstacle_detection/combined'])],
    #         parameters=[{
    #             'transform_tolerance': 0.01,
    #             'min_height': 0.0,
    #             'max_height': 1.0,
    #             'angle_min': -1.5708,  # -M_PI/2
    #             'angle_max': 1.5708,  # M_PI/2
    #             'angle_increment': 0.0087,  # M_PI/360.0
    #             'scan_time': 0.3333,
    #             'range_min': 0.45,
    #             'range_max': 4.0,
    #             'use_inf': False,
    #             'inf_epsilon': 1.0
    #         }],
    #         name='pointcloud_to_laserscan'
    # )

    # Note that this package WILL NOT start Gazebo
    # instead, when this launch file is executed it will wait for /spawn_entity
    # to be available. This will automatically be available after Gazebo is launched
    # from ROS2.
    return LaunchDescription(
        [
            robot_name_argument,
            x_position_argument,
            y_position_argument,
            z_position_argument,
            r_axis_argument,
            p_axis_argument,
            y_axis_argument,
            OpaqueFunction(function=__spawn_robot),
            wheels_driver_node,
            arms_driver_node,
            drums_driver_node,
            depth_img_to_ls
            #pc_to_ls
        ]
    )
