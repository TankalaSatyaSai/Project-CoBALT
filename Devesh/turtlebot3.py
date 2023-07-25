#! /usr/bin/env python3

import rospy
import math as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
t = 0

def callback_scan(data):
    global t
    t = 1
   

    
def publisher_node():
    rospy.init_node('move_turtlebot3')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback_scan)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        cmd_vel_msg = Twist()
        global t
        cmd_vel_msg.linear.x = 0.2*np.cos(10*t)
        cmd_vel_msg.linear.y = 0.2*np.sin(10*t)
        pub.publish(cmd_vel_msg)
        rate.sleep()


    






if __name__ == '__main__':
    try:
        publisher_node()
        
    except rospy.ROSInternalException:
        pass
