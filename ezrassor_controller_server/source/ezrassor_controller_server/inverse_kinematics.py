#! /usr/bin/env python

import rclpy 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random

def my_publisher():
    # control part

    rclpy.init_node('inverse_kinematics')
    control_publisher = rclpy.Publisher('/ezrassor/partial_autonomy', JointTrajectory, queue_size=10)

    while not rclpy.is_shutdown():
        
        msg = JointTrajectory()

        msg.header.stamp = rclpy.Time.now()
        msg.header.frame_id = ''
        msg.joint_names = ['joint12', 'joint23', 'joint34', 'joint45', 'joint56']

        point = JointTrajectoryPoint()
        j1 = 2 * (random.random() - 0.5)  # 0 - 1 -> -0.5 - 0.5
        j2 = 2 * (random.random() - 0.5)
        j3 = 2 * (random.random() - 0.5)
        j4 = 2 * (random.random() - 0.5)
        j5 = 2 * (random.random() - 0.5)

        point.positions = [j1, j2, j3, j4, j5]
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start = rclpy.Duration(5)

        msg.points.append( point )

        control_publisher.publish( msg )
        rclpy.loginfo( msg ) 


if __name__ == '__main__':

    my_publisher()
