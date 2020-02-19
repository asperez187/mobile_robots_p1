#!/usr/bin/env python
"""Setting interpreter"""

import sys
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Robot_bumper():
    """ Class of the control """
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.vel = Twist()
        self.sub_scan = rospy.Subscriber(self.robot_name+"/scan", LaserScan, callback_laser)
        self.pub_cmd = rospy.Publisher(self.robot_name+"/cmd_vel", Twist, queue_size=5)

    def callback_laser(self, msg):
        self.scan = msg.data

    def control(self):

def main():
    """Main function"""
    robot_class = Robot_bumper(str(sys.argv[1]))
    rospy.spin()

if __name__ == "__main__":
    # initialize node
    rospy.init_node('', anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        print("Shutting down publisher")
        pass