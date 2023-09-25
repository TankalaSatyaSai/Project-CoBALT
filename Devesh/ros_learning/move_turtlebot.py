#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
dist = 1

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "The distance to the obstacle is -  %s", msg.ranges[300])

    if msg.ranges[300] > dist:
        print("in")
        move.linear.x = 0.5
        move.angular.z = 0.0
    if msg.ranges[300] <= dist:
        print("in")
        move.linear.x = 0.0
        move.angular.z = 0.5
    pub.publish(move)

if __name__=='__main__':
  rospy.init_node("sub_node")
  sub = rospy.Subscriber("/scan", LaserScan, callback)
  pub = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
  rate = rospy.Rate(2)
  move = Twist()

rospy.spin()
