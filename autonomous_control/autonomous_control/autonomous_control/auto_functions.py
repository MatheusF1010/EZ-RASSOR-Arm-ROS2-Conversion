import rclpy
from .utility_functions import *
from .nav_functions import *
import time

def at_target(positionX, positionY, targetX, targetY, threshold):
    """Determine if the current position is within
    the desired threshold of the target position.
    """
    value = (targetX - threshold) < positionX < (targetX + threshold) and (
        targetY - threshold
    ) < positionY < (targetY + threshold)

    return value


def charge_battery(world_state, ros_util):
    """ Charge the rover's battery for a designated duration until battery is 100% """

    ros_util.publish_actions("stop", 0, 0, 0, 0)

    while world_state.battery < 100:
        time.sleep(0.1)
        world_state.battery += 3

    world_state.battery = 100


def auto_drive_location(world_state, ros_util, node, waypoint_server=None):
    """ Navigate to location. Avoid obstacles while moving toward location. """

    # Action server will print it's own info
    if waypoint_server is None:
        node.get_logger().info(
            "Auto-driving to {},{}...".format(world_state.target_location.x, world_state.target_location.y)
        )

    # Send feedback to waypoint client if being controlled by swarm controller
    preempted = False
    feedback = send_feedback(world_state, waypoint_server)

    node.get_logger().info("Setting front arms for travel!")
    # Set arms up for travel

    set_front_arm_angle(world_state, ros_util, 1.3)

    node.get_logger().info("Setting back arms for travel!")
    set_back_arm_angle(world_state, ros_util, 1.3)


    node.get_logger().info("Checking rover battery, hardware, and if it's flipped over")
    # Check rover battery, hardware, and if it's flipped over
    if self_check(world_state, ros_util) != 1:
        preempted = True

        # Cancel action server request is self check failed
        if waypoint_server is not None:
            waypoint_server.set_preempted()

        node.get_logger().info("Status check failed.")
        return feedback, preempted

    # Before we head towards our goal, turn to face it.
    # Get new heading angle relative to current heading
    node.get_logger().info("Calculating new heading angle...")
    new_heading_degrees = calculate_heading(world_state)
    angle2goal_radians = adjust_angle(
        world_state.heading, new_heading_degrees
    )

    # If our angle is less than zero, then we would expect a right turn
    # otherwise turn left.
    if angle2goal_radians < 0:
        direction = "right"
    else:
        direction = "left"

    node.get_logger().info("Our direction is: {}".format(direction))
    turn(new_heading_degrees, direction, world_state, ros_util)
    world_state.node.get_logger().info("Finished Turning To face goal.")
    ros_util.publish_actions("stop", 0, 0, 0, 0)

    # Main loop until location is reached
    while not at_target(
        world_state.positionX,
        world_state.positionY,
        world_state.target_location.x,
        world_state.target_location.y,
        ros_util.threshold,
    ):

        # Report current coordinate
        node.get_logger().info("My coords: ({},{})\nTarget coords: ({},{})".format(world_state.positionX, world_state.positionY, world_state.target_location.x, world_state.target_location.y))
        
        # Check that the waypoint client request hasnt been canceled
        if (
            waypoint_server is not None
            and waypoint_server.is_preempt_requested()
        ):
            preempted = True
            break

        # Set arms up for travel
        set_front_arm_angle(world_state, ros_util, 1.3)
        set_back_arm_angle(world_state, ros_util, 1.3)

        # Check rover battery, hardware, and if it's flipped over
        if self_check(world_state, ros_util) != 1:
            preempted = True
            # Cancel waypoint client request is self check failed
            if waypoint_server is not None:
                waypoint_server.set_preempted()

            node.get_logger().info("Status check failed.")
            break

        angle = get_turn_angle(world_state, ros_util)

        # If our angle is less than zero, then we would expect a right turn
        # otherwise turn left.
        direction = "right" if angle < 0 else "left"

        # Turn towards the direction chosen.
        turn(
            rel_to_abs(world_state.heading, angle),
            direction,
            world_state,
            ros_util,
        )

        # Move towards the direction chosen.
        move(ros_util.move_increment, world_state, ros_util)

        world_state.battery -= 0.1

        # Send feedback to waypoint action client
        feedback = send_feedback(world_state, waypoint_server)

    # Action server will print its own info
    if waypoint_server is None:
        node.get_logger().info("Destination reached!")

    ros_util.publish_actions("stop", 0, 0, 0, 0)
    return feedback, preempted


def auto_dig(world_state, ros_util, duration, node, waypoint_server=None):
    """
    Rotate both drums inward and dig
    for duration time in seconds.
    """

    feedback = send_feedback(world_state, waypoint_server)
    preempted = False

    # Check rover battery, hardware, and if it's flipped over
    if self_check(world_state, ros_util) != 1:
        if waypoint_server is not None:
            waypoint_server.set_preempted()

        preempted = True
        return feedback, preempted

    if waypoint_server is None:
        node.get_logger().info("Auto-digging for {} seconds...".format(duration))

    # @TODO: Set arms down for digging 
    # set_front_arm_angle(world_state, ros_util, -0.04)
    # set_back_arm_angle(world_state, ros_util, -0.04)

    # Dig for the desired duration
    t = 0
    direction = "forward"
    while t < duration * 40:
        # Swap between digging forward or backward every few seconds
        if t % 100 == 0:
            direction = "reverse" if direction == "forward" else "forward"
        node.get_logger().info("Publishing d, 0, 0, 1, 1 with i={}".format(t))
        ros_util.publish_actions(direction, 0, 0, 1, 1)

        t += 1
        ros_util.node.rate.sleep()

        world_state.battery -= 0.05

        # Send feedback to waypoint client if being controlled by swarm controller
        feedback = send_feedback(world_state, waypoint_server)

        if (
            waypoint_server is not None
            and waypoint_server.is_preempt_requested()
        ):
            preempted = True
            break

        if self_check(world_state, ros_util) != 1:
            if waypoint_server is not None:
                waypoint_server.set_preempted()

            preempted = True
            break

    ros_util.publish_actions("stop", 0, 0, 0, 0)
    return feedback, preempted


def auto_dock(world_state, ros_util, node):
    """ Dock with the hopper. """

    node.get_logger().info("Auto-returning to origin...")

    old_target_x = world_state.target_location.x
    old_target_y = world_state.target_location.y

    ros_util.threshold = 3

    world_state.target_location.x = float(0)
    world_state.target_location.y = float(0)

    auto_drive_location(world_state, ros_util, node)
    ros_util.threshold = 0.5

    world_state.target_location.x = old_target_x
    world_state.target_location.y = old_target_y


def auto_dump(world_state, ros_util, duration, node):
    """Rotate both drums inward and drive forward
    for duration time in seconds.
    """
    node.get_logger().info("Auto-dumping drum contents...")

    set_front_arm_angle(world_state, ros_util, 1.3)
    set_back_arm_angle(world_state, ros_util, 1.3)

    t = 0
    while t < duration * 40:
        if self_check(world_state, ros_util) != 1:
            return
        ros_util.publish_actions("stop", 0, 0, -1, -1)
        t += 1
        ros_util.node.rate.sleep()

    new_heading = world_state.heading = (world_state.heading + 180) % 360

    while not ((new_heading - 1) < world_state.heading < (new_heading + 1)):
        ros_util.publish_actions("left", 0, 0, 0, 0)
        ros_util.node.rate.sleep()

    while t < duration * 30:
        ros_util.publish_actions("stop", 0, 0, -1, -1)
        t += 1
        ros_util.node.rate.sleep()

    ros_util.publish_actions("stop", 0, 0, 0, 0)
