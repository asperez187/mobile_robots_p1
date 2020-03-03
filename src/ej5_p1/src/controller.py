#!/usr/bin/env python
"""Setting interpreter"""

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8

MIN_LASER = 0.2
LINEAR_SPEED = 0.1
ANGULAR_SPEED = 0.3

MAX_SPEED = 0.3
MAX_LASER = 0.5

m = MAX_SPEED/(MAX_LASER-(MIN_LASER-0.1))
n = MAX_SPEED - MAX_LASER*m

class RobotBumper():
    """ Class of the control """
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.state = 1 # 1 = gira, 2 = drive, 3 = stop 
        self.run = False

        for i in self.robot_name:
            if i.isdigit():
                self.robot_id = int(i)
                if self.robot_id == 1:
                    self.run = True
                    self.state = 2
        self.vel = Twist()
        self.odom = Odometry()
        self.scan = LaserScan()
        self.state_drive = "straight"
        
        self.sub_scan = rospy.Subscriber(self.robot_name+"/scan", LaserScan, self.callback_laser)
        self.sub_scan = rospy.Subscriber(self.robot_name+"/odom", Odometry, self.callback_odom)
        self.sub_control = rospy.Subscriber("/control", Int8, self.callback_control)

        self.pub_cmd = rospy.Publisher(self.robot_name+"/cmd_vel", Twist, queue_size=5)
        self.pub_control = rospy.Publisher("/control", Int8, queue_size=5)

        self.rate = rospy.Rate(100) # 0.1 seconds sleep
        self.bumper_right = 0
        self.bumper_left = 0
        self.bumper_middle = 0
        self.turn_value = 0

    def callback_control(self, msg):
        """ Para decidir que robot empieza """
        if self.robot_id == msg.data:
            self.run = True

    def callback_odom(self, msg):
        self.turn_value= msg.pose.pose.orientation.z

    def callback_laser(self, msg):
        """ Callback laser """
        self.scan = np.array(msg.ranges)
        self.bumper_left = np.amin(self.scan[30 : 90])
        self.bumper_middle = np.amin((np.amin(self.scan[330 : 359]), np.amin(self.scan[0 : 29])))
        self.bumper_right = np.amin(self.scan[270 : 329])

    def vel_stop(self):
        """ Stop """
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.pub_cmd.publish(self.vel)

    def vel_straight(self):
        if(self.bumper_middle > 0.5):
            self.vel.linear.x = MAX_SPEED
        else:
            self.vel.linear.x = m*self.bumper_middle + n
        
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

    def check_drive_state(self):
        """ Changing state of the program """
        rospy.loginfo("--------------")
        rospy.loginfo(self.bumper_left)
        rospy.loginfo(self.bumper_middle)
        rospy.loginfo(self.bumper_right)

        if self.bumper_left < MIN_LASER:
            self.state_drive = "turn_right"
        elif self.bumper_right < MIN_LASER:
            self.state_drive = "turn_left"
        elif self.bumper_middle < MIN_LASER:
            self.state_drive = "stop"
        else:
            self.state_drive = "straight"


    def drive(self):
        self.check_drive_state()
        # drive
        if self.state_drive == "stop":
            self.state = 3
            self.pub_control.publish(Int8(self.robot_id+1))
            self.vel_stop()
            self.run = False
        elif self.state_drive == "straight":
            self.vel_straight()
        elif self.state_drive == "turn_right":
            self.vel_right()
        elif self.state_drive == "turn_left":
            self.vel_left()
        elif self.state_drive == "backwards":
            self.vel_backwards()
        # sleep

    def turn_180(self):
        """ Turn 180 """
        rospy.loginfo(float(self.turn_value))
        if float(self.turn_value) < 0:
            self.state += 1
            self.vel_straight()
        else:
            self.vel_left()

    def stop(self):
        """ Stop """
        self.vel_stop()

    def control(self):
        """ Control node """
        while not rospy.is_shutdown():
            if self.run:
                rospy.loginfo(10*"-")
                rospy.loginfo("Robot id: "+ str(self.robot_id)+" Robot State :" + str(self.state))
                if self.state == 1:
                    self.turn_180()
                elif self.state == 2:
                    self.drive()
                elif self.state == 3:
                    self.stop()

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
