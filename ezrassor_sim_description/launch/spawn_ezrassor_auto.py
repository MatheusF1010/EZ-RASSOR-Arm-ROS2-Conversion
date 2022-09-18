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
import yaml


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
        pkg_ezrassor_sim_description, "urdf", "ezrassor_paver_arm_rover_auto.xacro.urdf"
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
    # CART
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

    load_partial_autonomy_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "-c",
            f"/{robot_name}/controller_manager",
            "--set-state",
            "start",
            "partial_autonomy_controller"
        ],
        output="screen",
    )

    load_claw_effort_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "-c",
            f"/{robot_name}/controller_manager",
            "--set-state",
            "start",
            "claw_effort_controller"
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
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_diff_drive_controller],
        #     )
        # ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_partial_autonomy_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_claw_effort_controller],
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_arm_back_velocity_controller],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_drum_back_velocity_controller],
        #     )
        # ),
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
    paver_arm_auto_driver_node = Node(
        package="ezrassor_sim_description",
        executable="paver_arm_auto_driver",
        namespace=LaunchConfiguration("robot_name"),
        output={"both": "screen"},
    )
    depth_img_to_ls = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
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



    # GDB Debug Option
        # Not important right now

    # Verbose Mode Option right now
        # Not important

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
    # with open(urdf_file, 'r') as infp:
    #     robot_urdf = infp.read()
    robot_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file,
            " ",
        ]
    )

    # loading ros_controllers.yaml
    controller_yaml_file = os.path.join(
        pkg_ezrassor_arm_moveit_ros2, "config", "ros_controllers.yaml"
    )
    with open(controller_yaml_file, 'r') as file:
        controller = yaml.safe_load(file)

    # loading ompl_planning.yaml
    ompl_planning_yaml_file = os.path.join(
        pkg_ezrassor_arm_moveit_ros2, "config", "ompl_planning.yaml"
    )
    with open(ompl_planning_yaml_file, 'r') as file:
        ompl_planning = yaml.safe_load(file)

    # loading joint_limits.yaml
    joint_limits_yaml_file = os.path.join(
        pkg_ezrassor_arm_moveit_ros2, "config", "joint_limits.yaml"
    )
    with open(joint_limits_yaml_file, 'r') as file:
        joint_limits = yaml.safe_load(file)

    # loading cartesian_limits.yaml
    cartesian_limits_yaml_file = os.path.join(
        pkg_ezrassor_arm_moveit_ros2, "config", "cartesian_limits.yaml"
    )
    with open(cartesian_limits_yaml_file, 'r') as file:
        cartesian_limits = yaml.safe_load(file)

    # loading kinematics.yaml
    kinematics_yaml_file = os.path.join(
        pkg_ezrassor_arm_moveit_ros2, "config", "kinematics.yaml"
    )
    with open(kinematics_yaml_file, 'r') as file:
        kinematics = yaml.safe_load(file)

    # loading sensors_3d.yaml
    sensors_3d_yaml_file = os.path.join(
        pkg_ezrassor_arm_moveit_ros2, "config", "sensors_3d.yaml"
    )
    with open(sensors_3d_yaml_file, 'r') as file:
        sensors_3d = yaml.safe_load(file)

    # loading ezrassor.srdf
    ezrassor_srdf_file = os.path.join(
        pkg_ezrassor_arm_moveit_ros2, "config", "ezrassor.srdf"
    )
    with open(ezrassor_srdf_file, 'r') as infp:
        robot_desc = infp.read()

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
        # namespace='/ezrassor',
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
            # wheels_driver_node,
            # arms_driver_node,
            # drums_driver_node,
            paver_arm_auto_driver_node,
            depth_img_to_ls,
            # move_group
        ]
    )
