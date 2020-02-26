
#!/usr/bin/env python
"""Setting interpreter"""

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

MIN_BUMP_DISTANCE = 0.2
LINEAR_SPEED = 0.15
ANGULAR_SPEED = 0.5

class RobotBumper():
    """ Class of the control """
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.vel = Twist()
        self.scan = LaserScan()
        self.state = "stop"
        self.sub_scan = rospy.Subscriber(self.robot_name+"/scan", LaserScan, self.callback_laser)
        self.pub_cmd = rospy.Publisher(self.robot_name+"/cmd_vel", Twist, queue_size=5)
        self.rate = rospy.Rate(1) # 0.1 seconds sleep
        self.bumper_right = 0
        self.bumper_left = 0
        self.bumper_middle = 0


    def callback_laser(self, msg):
        """ Callback laser """
        self.scan = np.array(msg.ranges)
        self.bumper_left = np.amin(self.scan[30 : 90])
        self.bumper_middle = np.amin((np.amin(self.scan[330 : 359]),np.amin(self.scan[0 : 29])))
        self.bumper_right = np.amin(self.scan[270 : 329])

    def vel_stop(self):
        """ Stop """
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.pub_cmd.publish(self.vel)

    def vel_straight(self):
        """ Go straight """
        self.vel.linear.x = LINEAR_SPEED
        self.vel.angular.z = 0.0
        self.pub_cmd.publish(self.vel)

    def vel_backwards(self):
        """ Go backwards """
        self.vel.linear.x = -LINEAR_SPEED
        self.vel.angular.z = 0.0
        self.pub_cmd.publish(self.vel)

    def vel_right(self):
        """ Go right """
        self.vel.linear.x = 0.0
        self.vel.angular.z = -ANGULAR_SPEED
        self.pub_cmd.publish(self.vel)

    def vel_left(self):
        """ Go left """
        self.vel.linear.x = 0.0
        self.vel.angular.z = ANGULAR_SPEED
        self.pub_cmd.publish(self.vel)

    def check_state(self):
        """ Changing state of the program """
        rospy.loginfo("--------------")
        rospy.loginfo(self.bumper_left)
        rospy.loginfo(self.bumper_middle)
        rospy.loginfo(self.bumper_right)

        if self.bumper_left < MIN_BUMP_DISTANCE:
            self.state = "turn_right"
        elif self.bumper_right < MIN_BUMP_DISTANCE:
            self.state = "turn_left"
        elif self.bumper_middle < MIN_BUMP_DISTANCE:
            self.state = "backwards"
        else:
            self.state = "straight"

    def control(self):
        """ Control node """
        while not rospy.is_shutdown():
            self.check_state()
            # move accordingly to the state
            if self.state == "stop":
                self.vel_stop()
            elif self.state == "straight":
                self.vel_straight()
            elif self.state == "turn_right":
                self.vel_right()
            elif self.state == "turn_left":
                self.vel_left()
            elif self.state == "backwards":
                self.vel_backwards()
            # sleep
            self.rate.sleep()


def main():
    """Main function"""
    robot_class = RobotBumper(str(sys.argv[1]))
    robot_class.control()
    rospy.spin()

if __name__ == "__main__":
    # initialize node
    rospy.init_node('controller', anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        print("Shutting down publisher")
        pass