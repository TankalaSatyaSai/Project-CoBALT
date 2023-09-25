#!usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def laserfun(laser= LaserScan):
  rospy.loginfo(laser)

  
if __name__=='__main__':
  rospy.init_node('sensor_msg')
  rospy.Subscriber('turtle/laser_scan', LaserScan, callback=laserfun)
