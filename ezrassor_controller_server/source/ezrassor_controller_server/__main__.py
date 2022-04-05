"""Execute a ROS node using the ezrassor_controller_server module.

This node listens for HTTP messages over the wire that can be translated into
commands for the EZRASSOR. The translated commands are published to ROS topics
that the EZRASSOR understands.
"""
import ezrassor_controller_server as server
import flask
import geometry_msgs.msg
import rclpy
import std_msgs.msg


NODE = "controller_server"
WHEEL_ACTIONS_TOPIC = "wheel_actions"
#FRONT_ARM_ACTIONS_TOPIC = "front_arm_actions"
BACK_ARM_ACTIONS_TOPIC = "back_arm_actions"
#FRONT_DRUM_ACTIONS_TOPIC = "front_drum_actions"
BACK_DRUM_ACTIONS_TOPIC = "back_drum_actions"
ROUTINE_ACTIONS_TOPIC = "routine_actions"

PAVER_ARM_JOINT_1_ACTIONS_TOPIC = "paver_arm_joint_1_actions"
PAVER_ARM_JOINT_2_ACTIONS_TOPIC = "paver_arm_joint_2_actions"
PAVER_ARM_JOINT_3_ACTIONS_TOPIC = "paver_arm_joint_3_actions"
PAVER_ARM_JOINT_4_ACTIONS_TOPIC = "paver_arm_joint_4_actions"
PAVER_ARM_JOINT_5_ACTIONS_TOPIC = "paver_arm_joint_5_actions"
PAVER_ARM_CLAW_ACTIONS_TOPIC = "paver_arm_claw_actions"

QUEUE_SIZE = 10


def main(passed_args=None):
    """Main entry point for the ROS node."""
    try:
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

        # Create publishers for each type of action.
        wheel_actions_publisher = node.create_publisher(
            geometry_msgs.msg.Twist,
            WHEEL_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        # front_arm_actions_publisher = node.create_publisher(
        #     std_msgs.msg.Float32,
        #     FRONT_ARM_ACTIONS_TOPIC,
        #     QUEUE_SIZE,
        # )
        back_arm_actions_publisher = node.create_publisher(
            std_msgs.msg.Float32,
            BACK_ARM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        # back_front_actions_publisher = node.create_publisher(
        #     std_msgs.msg.Float32,
        #     FRONT_DRUM_ACTIONS_TOPIC,
        #     QUEUE_SIZE,
        # )
        back_drum_actions_publisher = node.create_publisher(
            std_msgs.msg.Float32,
            BACK_DRUM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        routine_actions_publisher = node.create_publisher(
            std_msgs.msg.Int8,
            ROUTINE_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        paver_arm_joint_1_publisher = node.create_publisher(
            std_msgs.msg.Int8,
            PAVER_ARM_JOINT_1_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        paver_arm_joint_2_publisher = node.create_publisher(
            std_msgs.msg.Int8,
            PAVER_ARM_JOINT_2_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        paver_arm_joint_3_publisher = node.create_publisher(
            std_msgs.msg.Int8,
            PAVER_ARM_JOINT_3_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        paver_arm_joint_4_publisher = node.create_publisher(
            std_msgs.msg.Int8,
            PAVER_ARM_JOINT_4_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        paver_arm_joint_5_publisher = node.create_publisher(
            std_msgs.msg.Int8,
            PAVER_ARM_JOINT_5_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        paver_arm_claw_publisher = node.create_publisher(
            std_msgs.msg.Int8,
            PAVER_ARM_CLAW_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )

        def process_request(request):
            """Callback to create and publish a command from a request."""
            server.verify(request)
            command = server.create_command(request)

            if command.wheel_action is not None:
                wheel_action = geometry_msgs.msg.Twist()
                wheel_action.linear.x = command.wheel_action.linear_x
                wheel_action.angular.z = command.wheel_action.angular_z
                wheel_actions_publisher.publish(wheel_action)

            # if command.front_arm_action is not None:
            #     front_arm_action = std_msgs.msg.Float32()
            #     front_arm_action.data = command.front_arm_action.value
            #     front_arm_actions_publisher.publish(front_arm_action)

            if command.back_arm_action is not None:
                back_arm_action = std_msgs.msg.Float32()
                back_arm_action.data = command.back_arm_action.value
                back_arm_actions_publisher.publish(back_arm_action)

            # if command.front_drum_action is not None:
            #     front_drum_action = std_msgs.msg.Float32()
            #     front_drum_action.data = command.front_drum_action.value
            #     back_front_actions_publisher.publish(front_drum_action)

            if command.back_drum_action is not None:
                back_drum_action = std_msgs.msg.Float32()
                back_drum_action.data = command.back_drum_action.value
                back_drum_actions_publisher.publish(back_drum_action)

            if command.routine_action is not None:
                routine_action = std_msgs.msg.Int8()
                routine_action.data = command.routine_action.value
                routine_actions_publisher.publish(routine_action)

            if command.paver_arm_joint_1_action is not None:
                paver_arm_joint_1_action = std_msgs.msg.Int8()
                paver_arm_joint_1_action.data = command.paver_arm_joint_1_action.value
                paver_arm_joint_1_publisher.publish(paver_arm_joint_1_action)
            
            if command.paver_arm_joint_2_action is not None:
                paver_arm_joint_2_action = std_msgs.msg.Int8()
                paver_arm_joint_2_action.data = command.paver_arm_joint_2_action.value
                paver_arm_joint_2_publisher.publish(paver_arm_joint_2_action)

            if command.paver_arm_joint_3_action is not None:
                paver_arm_joint_3_action = std_msgs.msg.Int8()
                paver_arm_joint_3_action.data = command.paver_arm_joint_3_action.value
                paver_arm_joint_3_publisher.publish(paver_arm_joint_3_action)

            if command.paver_arm_joint_4_action is not None:
                paver_arm_joint_4_action = std_msgs.msg.Int8()
                paver_arm_joint_4_action.data = command.paver_arm_joint_4_action.value
                paver_arm_joint_4_publisher.publish(paver_arm_joint_4_action)

            if command.paver_arm_joint_5_action is not None:
                paver_arm_joint_5_action = std_msgs.msg.Int8()
                paver_arm_joint_5_action.data = command.paver_arm_joint_5_action.value
                paver_arm_joint_5_publisher.publish(paver_arm_joint_5_action)

            if command.paver_arm_claw_action is not None:
                paver_arm_claw_action = std_msgs.msg.Int8()
                paver_arm_claw_action.data = command.paver_arm_claw_action.value
                paver_arm_claw_publisher.publish(paver_arm_claw_action)


        # Create a Flask app to serve HTTP requests.
        app = flask.Flask(__name__)

        @app.route("/", methods=["POST"])
        def handle_request():
            """Handle HTTP requests and log any errors.

            This function is the glue between Flask/HTTP and ROS business logic
            in this package.
            """
            try:
                process_request(flask.request.get_json())

                return {"status": 200}
            except server.VerificationError as error:
                node.get_logger().error(str(error))

                return {"status": 400}

        # Run the app! Note that we don't spin the ROS node here. Only nodes
        # containing subscribers must be spun.
        app.run()
    except KeyboardInterrupt:
        pass
