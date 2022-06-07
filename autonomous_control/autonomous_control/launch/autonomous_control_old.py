# Launch autonomous control for the EZ-RASSOR 2.0
#      Written by Nathan Kurelo Wilk -->

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
#from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Create an autonomous control node with a launch description."""
    # Create arguments for launch file

    wheel_instructions_topic_argument = DeclareLaunchArgument(
        'wheel_instructions_topic',
        default_value='wheel_instructions',
        description='Wheel instructions topic'
    )

    front_arm_instructions_topic_argument = DeclareLaunchArgument(
        'front_arm_instructions_topic',
        default_value='front_arm_instructions',
        description='Front arm instructions topic'
    )

    back_arm_instructions_topic_argument = DeclareLaunchArgument(
        'back_arm_instructions_topic',
        default_value='back_arm_instructions',
        description='Back arm instructions topic'
    )

    front_drum_instructions_topic_argument = DeclareLaunchArgument(
        'front_drum_instructions_topic',
        default_value='front_drum_instructions',
        description='Front drum instructions topic'
    )

    back_drum_instructions_topic_argument = DeclareLaunchArgument(
        'back_drum_instructions_topic',
        default_value='back_drum_instructions',
        description='Back drum instructions topic'
    )

    digsite_x_coord_argument = DeclareLaunchArgument(
        "digsite_x_coord",
        default_value="10",
        description="The x coordinate of the digsite"
    )

    digsite_y_coord_argument = DeclareLaunchArgument(
        "digsite_y_coord",
        default_value="10",
        description="The y coordinate of the digsite"
    )

    spawn_x_coord_argument = DeclareLaunchArgument(
        "spawn_x_coord",
        default_value="0",
        description="The x coordinate of the spawn point"
    )

    spawn_y_coord_argument = DeclareLaunchArgument(
        "spawn_y_coord",
        default_value="0",
        description="The y coordinate of the spawn point"
    )

    max_linear_velocity_argument = DeclareLaunchArgument(
        "max_linear_velocity",
        default_value="1.0",
        description="The maximum linear velocity of the robot"
    )

    max_angular_velocity_argument = DeclareLaunchArgument(
        "max_angular_velocity",
        default_value="1.0",
        description="The maximum angular velocity of the robot"
    )

    enable_real_odometry_argument = DeclareLaunchArgument(
        "enable_real_odometry",
        default_value="false",
        description="Whether to enable real odometry"
    )

    swarm_control_argument = DeclareLaunchArgument(
        "swarm_control",
        default_value="false",
        description="Whether to enable swarm control"
    )

    obstacle_threshold_argument = DeclareLaunchArgument(
        "obstacle_threshold",
        default_value="4.0",
        description="The distance threshold for obstacle detection"
    )

    obstacle_buffer_argument = DeclareLaunchArgument(
        "obstacle_buffer",
        default_value="1.5",
        description="The buffer distance for obstacle detection"
    )

    move_increment_argument = DeclareLaunchArgument(
        "move_increment",
        default_value="3.0",
        description="The distance to move in each direction"
    )

    max_obstacle_angle_argument = DeclareLaunchArgument(
        "max_obstacle_angle",
        default_value="45.0",
        description="The maximum angle to turn to avoid an obstacle"
    )

    min_hole_diameter_argument = DeclareLaunchArgument(
        "min_hole_diameter",
        default_value="3.0",
        description="The minimum diameter of a hole to be considered"
    )

    enable_park_ranger_argument = DeclareLaunchArgument(
        "enable_park_ranger",
        default_value="false",
        description="Whether to enable the park ranger"
    )

    world_argument = DeclareLaunchArgument(
        "world",
        default_value="default",
        description="The world to load"
    )

    # Launch the autonomous control node
    # ezrassor_autonomous_control_node = Node(
    #     package="ezrassor_autonomous_control",
    #     executable="autonomous_control",
    #     parameters=[
    #         {
    #             "digsite_x_coord": LaunchConfiguration("digsite_x_coord"),
    #             "digsite_y_coord": LaunchConfiguration("digsite_y_coord"),
    #             "spawn_x_coord": LaunchConfiguration("spawn_x_coord"),
    #             "spawn_y_coord": LaunchConfiguration("spawn_y_coord"),
    #             "max_linear_velocity": LaunchConfiguration("max_linear_velocity"),
    #             "max_angular_velocity": LaunchConfiguration("max_angular_velocity"),
    #             "enable_real_odometry": LaunchConfiguration("enable_real_odometry"),
    #             "wheel_instructions_topic": LaunchConfiguration("wheel_instructions_topic"),
    #             "front_arm_instructions_topic": LaunchConfiguration("front_arm_instructions_topic"),
    #             "back_arm_instructions_topic": LaunchConfiguration("back_arm_instructions_topic"),
    #             "front_drum_instructions_topic": LaunchConfiguration("front_drum_instructions_topic"),
    #             "back_drum_instructions_topic": LaunchConfiguration("back_drum_instructions_topic"),
    #             "swarm_control": LaunchConfiguration("swarm_control"),
    #             "obstacle_threshold": LaunchConfiguration("obstacle_threshold"),
    #             "obstacle_buffer": LaunchConfiguration("obstacle_buffer"),
    #             "move_increment": LaunchConfiguration("move_increment")
    #         },
    #     ],
    #     output={"both": "screen"},
    # )

    # #Launch an image_view node to display the camera feed
    # image_view_node = Node(
    #     package="image_view",
    #     executable="image_view",
    #     output={"both": "screen"},
    #     # Add a remapping from 'image' to 'depth/image_raw'
    #     remappings=[("image", "depth/image_raw")],
    # )

    # Launch the obstacle detection node
    obstacle_detection_node = Node(
        package="autonomous_control", #ezrassor_autonomous_control
        executable="obstacle_detection",
        parameters=[
            {
                "max_angle": LaunchConfiguration("max_obstacle_angle"),
                "max_obstacle_dist": LaunchConfiguration("obstacle_threshold"),
                "min_hole_diameter": LaunchConfiguration("min_hole_diameter"),
            }
        ],
        output={"both": "screen"},
    )

    # # Create a groupping if enable_park_ranger is true
    # if LaunchConfiguration("enable_park_ranger") == "true":
    #     park_ranger_group = GroupAction([
    #         Node(
    #             package="ezrassor_autonomous_control",
    #             executable="park_ranger",
    #             parameters=[{"world": LaunchConfiguration("world")}],
    #             output={"both": "screen"},
    #         )
    #     ])
    
    # # Create a groupping if enable_real_odometry is true
    # # include the rgbd_odometry node from the rtabmap_ros package
    # # include the node ekf_localization_node from the robot_localization package
    # if LaunchConfiguration("enable_real_odometry") == "true":

    #     # Define the config file
    #     config = os.path.join(
    #         get_package_share_directory('ezrassor_autonomous_control'),
    #         'config',
    #         'ekf_template.yaml'
    #     )
        
    #     real_odometry_group = GroupAction([
    #         Node(
    #             package="rtabmap_ros",
    #             executable="rgbd_odometry", #visual_odometry
    #             parameters=[
    #                 {
    #                     "frame_id": "depth_camera_optical_frame",
    #                     "publish_tf": "false",
    #                     "publish_null_when_lost": "false",
    #                     "Odom/ResetCountdown": "1",
    #                 }
    #             ],
    #             remappings=[
    #                 ("rgb/image", "color/image_raw"),
    #                 ("rgb/camera_info", "color/camera_info"),
    #                 ("depth/image", "depth/image_raw"),
    #                 ("odom", "visual_odom/odom"),
    #                 ("odom_info", "visual_odom/odom_info"),
    #                 ("odom_last_frame", "visual_odom/odom_last_frame"),
    #                 ("odom_local_map", "visual_odom/odom_local_map"),
    #             ],
    #             output={"both": "screen"},
    #         ),
    #         # TODO: rosparam load lookup
    #         # rosparameter from yaml file

    #         Node(
    #             package="robot_localization",
    #             executable="ekf_localization_node",
    #             name="ekf_se",
    #             # add a rosparam command to load the ekf_params.yaml file
    #             parameters=[config],
    #             output={"both": "screen"},
    #         )
    #     ])

    return LaunchDescription(
        [
            digsite_x_coord_argument,
            digsite_y_coord_argument,
            spawn_x_coord_argument,
            spawn_y_coord_argument,
            max_linear_velocity_argument,
            max_angular_velocity_argument,
            enable_real_odometry_argument,
            swarm_control_argument,
            obstacle_threshold_argument,
            obstacle_buffer_argument,
            move_increment_argument,
            max_obstacle_angle_argument,
            min_hole_diameter_argument,
            enable_park_ranger_argument,
            world_argument,
            # ezrassor_autonomous_control_node,
            # image_view_node,
            obstacle_detection_node,
            # park_ranger_group,
            # real_odometry_group,
        ]
    )
