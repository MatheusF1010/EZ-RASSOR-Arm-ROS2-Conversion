# Import necessary dependencies
from unittest.mock import DEFAULT
import rclpy
from rclpy.node import Node

# import float64 multiarray
from std_msgs.msg import Float64MultiArray, Int32, Float64
import time

class SimulationRoutine(Node):
    DEFAULT_ROBOT_ALONE = "ezrassor"
    ROBOT_NAMES = ["ezrassor", "ezrassor_2", "ezrassor_3", "ezrassor_4"]
    PUBLISHERS = {}
    TIME_DELAY = 2
    ARM_SPEED = 0.5

    def __init__(self):

        super().__init__("simulation_interact")
        self.get_logger().info('Created simulation_interact node')

        self.interact = self.create_subscription(Int32, "/interact", self.on_interaction, 10)
        self.get_logger().info('Created interact subs')

        self.timed = self.create_subscription(Float64, "/change_interaction_time", self.change_time, 10)
        self.get_logger().info('Created timed subs')

        self.arm_vel = self.create_subscription(Float64, "/change_arm_velocity", self.change_arm_vel, 10)
        self.get_logger().info('Created arm_vel subs')

        # Set up publishers for each robot
        for each_robot in self.ROBOT_NAMES:
            if not each_robot in self.PUBLISHERS:
                self.PUBLISHERS[each_robot] = {}
            self.PUBLISHERS[each_robot]['arm_f'] = self.create_publisher(Float64MultiArray, "{}/arm_front_velocity_controller/commands".format(each_robot), 10)
            self.PUBLISHERS[each_robot]['arm_b'] = self.create_publisher(Float64MultiArray, "{}/arm_back_velocity_controller/commands".format(each_robot), 10)
            self.PUBLISHERS[each_robot]['drum_f'] = self.create_publisher(Float64MultiArray, "{}/drum_front_velocity_controller/commands".format(each_robot), 10)
            self.PUBLISHERS[each_robot]['drum_b'] = self.create_publisher(Float64MultiArray, "{}/drum_back_velocity_controller/commands".format(each_robot), 10)

    def on_interaction(self, int32msg):
        mode = int32msg.data
        if mode == 0:
            self.get_logger().info('Interaction mode is off')
            return
        elif mode == 1:
            self.arm_routine()
    
    def change_time(self, float64msg):
        self.TIME_DELAY = float64msg.data
        self.get_logger().info('Time delay is now {}'.format(self.TIME_DELAY))
    
    def change_arm_vel(self, float64msg):
        self.ARM_SPEED = float64msg.data
        self.get_logger().info('Arm speed is now {}'.format(self.ARM_SPEED))

    def arm_routine(self):
        # Do this five times:
        self.raise_lower_both_arms()
        self.dance_front_then_back()

    def dance_front_then_back(self):
        self.raise_front_arm()
        time.sleep(self.TIME_DELAY)
        self.stop_front_arm()

        self.raise_back_arm()
        self.lower_front_arm()
        time.sleep(self.TIME_DELAY)
        self.stop_back_arm()
        self.stop_front_arm()
        time.sleep(self.TIME_DELAY)
        self.lower_back_arm()
        time.sleep(self.TIME_DELAY)
        self.stop_back_arm()

    def raise_lower_both_arms(self):
        self.raise_both_arms()
        time.sleep(self.TIME_DELAY)
        self.stop_both_arms()
        self.lower_both_arms()
        time.sleep(self.TIME_DELAY)
        self.stop_both_arms()
    
    def raise_front_arm(self):
        self.get_logger().info('Raising front arms for {}'.format(self.DEFAULT_ROBOT_ALONE))
        arm_f_msg = Float64MultiArray()
        arm_f_msg.data = [self.ARM_SPEED]
        self.PUBLISHERS[self.DEFAULT_ROBOT_ALONE]['arm_f'].publish(arm_f_msg)

    def raise_back_arm(self):
        self.get_logger().info('Raising back arms for {}'.format(self.DEFAULT_ROBOT_ALONE))
        arm_b_msg = Float64MultiArray()
        arm_b_msg.data = [self.ARM_SPEED]
        self.PUBLISHERS[self.DEFAULT_ROBOT_ALONE]['arm_b'].publish(arm_b_msg)
    
    def stop_front_arm(self):
        self.get_logger().info('Stopping front arms for {}'.format(self.DEFAULT_ROBOT_ALONE))
        arm_f_msg = Float64MultiArray()
        arm_f_msg.data = [0.0]
        self.PUBLISHERS[self.DEFAULT_ROBOT_ALONE]['arm_f'].publish(arm_f_msg)
    
    def stop_back_arm(self):
        self.get_logger().info('Stopping back arms for {}'.format(self.DEFAULT_ROBOT_ALONE))
        arm_b_msg = Float64MultiArray()
        arm_b_msg.data = [0.0]
        self.PUBLISHERS[self.DEFAULT_ROBOT_ALONE]['arm_b'].publish(arm_b_msg)
    
    def lower_front_arm(self):
        self.get_logger().info('Lowering front arms for {}'.format(self.DEFAULT_ROBOT_ALONE))
        arm_f_msg = Float64MultiArray()
        arm_f_msg.data = [-self.ARM_SPEED]
        self.PUBLISHERS[self.DEFAULT_ROBOT_ALONE]['arm_f'].publish(arm_f_msg)
    
    def lower_back_arm(self):
        self.get_logger().info('Lowering back arms for {}'.format(self.DEFAULT_ROBOT_ALONE))
        arm_b_msg = Float64MultiArray()
        arm_b_msg.data = [-self.ARM_SPEED]
        self.PUBLISHERS[self.DEFAULT_ROBOT_ALONE]['arm_b'].publish(arm_b_msg)

    def raise_both_arms(self):
        self.get_logger().info('Raising arms for {}'.format(self.DEFAULT_ROBOT_ALONE))
        arm_f_msg = Float64MultiArray()
        arm_f_msg.data = [self.ARM_SPEED]
        arm_b_msg = Float64MultiArray()
        arm_b_msg.data = [self.ARM_SPEED]
        self.PUBLISHERS[self.DEFAULT_ROBOT_ALONE]['arm_f'].publish(arm_f_msg)
        self.PUBLISHERS[self.DEFAULT_ROBOT_ALONE]['arm_b'].publish(arm_b_msg)
    
    def lower_both_arms(self):
        self.get_logger().info('Lowering arms for {}'.format(self.DEFAULT_ROBOT_ALONE))
        arm_f_msg = Float64MultiArray()
        arm_f_msg.data = [-self.ARM_SPEED]
        arm_b_msg = Float64MultiArray()
        arm_b_msg.data = [-self.ARM_SPEED]
        self.PUBLISHERS[self.DEFAULT_ROBOT_ALONE]['arm_f'].publish(arm_f_msg)
        self.PUBLISHERS[self.DEFAULT_ROBOT_ALONE]['arm_b'].publish(arm_b_msg)
    
    def stop_both_arms(self):
        self.get_logger().info('Stopping arms for {}'.format(self.DEFAULT_ROBOT_ALONE))
        arm_f_msg = Float64MultiArray()
        arm_f_msg.data = [0.0]
        arm_b_msg = Float64MultiArray()
        arm_b_msg.data = [0.0]
        self.PUBLISHERS[self.DEFAULT_ROBOT_ALONE]['arm_f'].publish(arm_f_msg)
        self.PUBLISHERS[self.DEFAULT_ROBOT_ALONE]['arm_b'].publish(arm_b_msg)
    
def main(args=None):
    rclpy.init(args=args)

    sim_routine = SimulationRoutine()

    rclpy.spin(sim_routine)
    sim_routine.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()