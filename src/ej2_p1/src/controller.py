#!/usr/bin/env python
"""Setting interpreter"""

import sys
import rospy
import numpy as np
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Robot_bumper():
    """ Class of the control """
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.vel = Twist()
        self.scan = LaserScan()
        self.state = "straight"
        self.sub_scan = rospy.Subscriber(self.robot_name+"/scan", LaserScan, self.callback_laser)
        self.pub_cmd = rospy.Publisher(self.robot_name+"/cmd_vel", Twist, queue_size=5)
        self.rate = rospy.Rate(10) # 0.1 seconds sleep

    def callback_laser(self, msg):
        """ Callback laser """
        self.scan = np.array(msg.ranges)
        # self.bumper = msg.ranges[:, 120]
        self.bumper_left = np.mean(self.scan[90:149])
        self.bumper_middle = np.mean(self.scan[150 : 209])
        self.bumper_right = np.mean(self.scan[210 : 269])

    def vel_straight(self):
        """ Go straight """
        self.vel.linear.x = 0.3
        self.vel.angular.z = 0.0
        self.pub_cmd.publish(self.vel)

    def vel_right(self):
        """ Go right """
        self.vel.linear.x = 0.0
        self.vel.angular.z = -0.3
        self.pub_cmd.publish(self.vel)

    def vel_left(self):
        """ Go left """
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.3
        self.pub_cmd.publish(self.vel)

    def control(self):
        """ Control node """
        while not rospy.is_shutdown():
            if self.state == "straight":
                self.vel_straight()
            elif self.state == "turn_right":
                self.vel_right()
            elif self.state == "turn_left":
                self.vel_left()


def main():
    """Main function"""
    robot_class = Robot_bumper(str(sys.argv[1]))
    robot_class.vel_straight()
    rospy.spin()

if __name__ == "__main__":
    # initialize node
    rospy.init_node('controller', anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        print("Shutting down publisher")
        pass