#!/usr/bin/env python
"""Setting interpreter"""

import sys
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs import LaserScan
from std_msgs.msg import Int8

def main():
  """Main function"""
  rospy.spin()

if __name__ == "__main__":
  # initialize node
  rospy.init_node('', anonymous=True)
  try:
    main()
  except rospy.ROSInterruptException:
    print("Shutting down publisher")
    pass