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
        self.sub_scan = rospy.Subscriber(self.robot_name+"/scan", LaserScan, self.callback_laser)
        self.pub_cmd = rospy.Publisher(self.robot_name+"/cmd_vel", Twist, queue_size=5)
        self.rate = rospy.Rate(10) # 0.1 seconds sleep

    def callback_laser(self, msg):
        """ Callback laser """
        self.scan = np.array(msg.ranges)
        # self.bumper = msg.ranges[:, 120]
        self.bumper_left = np.mean(self.scan[:119])
        self.bumper_middle = np.mean(self.scan[120 : 239])
        self.bumper_right = np.mean(self.scan[240:])

    def vel_straight(self):
        """ Go straight """
        self.vel.linear.x = 0.3
        self.vel.angular.z = 0.0
        self.pub_cmd.publish(self.vel)
    

    def control(self):
        """ hola """
        # while not rospy.is_shutdown():


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