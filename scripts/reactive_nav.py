#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random
import numpy as np
import threading

# The Two-Face algorithm:
# When the robot encounters an obstacle and is stuck, it
# randomly selects the next direction (left or right) using a coin flip (a random boolean).
# This chosen direction is maintained for 30 seconds due to certain technical issues 
# that cause the robot to "hesitate" (i.e., switch directions while remaining in place).
# The algorithm's name is inspired by the Batman character Two-Face, who uses a coin flip
# to determine his actions.

class Coin:
    def __init__(self):
        self.heads = False 
        self.tails = True
        self.flipped = False
        self.timer = None  

    def flip(self):
        if not self.flipped:
            self.heads = random.choice([True, False])
            self.tails = not self.heads
            self.flipped = True
        
        if self.timer is not None:  # These lines reset the coin: if 0.5 seconds pass without detecting an obstacle,
                                    # the coin is reset, and it will be flipped again upon encountering the next obstacle.
            self.timer.cancel()

        self.timer = threading.Timer(1, self.reset)
        self.timer.start()

    def reset(self):
        rospy.loginfo("Coin reset")
        self.flipped = False

coin = Coin()

class ReactiveNav:
    def __init__(self):
        rospy.init_node('reactive_nav', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.forward_speed = 2  
        self.turn_speed = 2  

        self.closest_obstacle_distance = float('inf')
        self.closest_obstacle_angle = 0.0

    def scan_callback(self, scan_data):
        self.closest_obstacle_distance = min(scan_data.ranges)
        self.closest_obstacle_angle = scan_data.ranges.index(self.closest_obstacle_distance)

        # rospy.loginfo("Scan data received.")
        # rospy.loginfo("Ranges: %s", scan_data.ranges)
        # rospy.loginfo("Min range: %s", np.nanmin(scan_data.ranges))

    def move_robot(self):  # A test function to make the robot rotate in place to test the laser scan
        move_cmd = Twist()

        # Simple reactive strategy: if the robot is too close to an obstacle, it turns; otherwise, it moves forward
        if self.closest_obstacle_distance < 1:  # Threshold distance to obstacle
            coin.flip()
            move_cmd.angular.z = -self.turn_speed * coin.heads + self.turn_speed * coin.tails  # Heads leads to a negative angle, tails to a positive angle
            move_cmd.linear.x = 0.0

            rospy.loginfo("Encountered an obstacle")
        else:
            move_cmd.linear.x = self.forward_speed
            move_cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(move_cmd)

    def run(self):
        rospy.loginfo("Running")
        while not rospy.is_shutdown():
            self.move_robot()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        nav = ReactiveNav()
        nav.run()
    except rospy.ROSInterruptException:
        pass

