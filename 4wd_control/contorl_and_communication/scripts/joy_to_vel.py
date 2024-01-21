#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTeleop:
    def __init__(self):
        rospy.init_node('joy_teleop')

        rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def joy_callback(self, joy_msg):
        
        linear_x = joy_msg.axes[1]
        linear_y = joy_msg.axes[0]  
        angular_z = joy_msg.axes[3] 

        # Create Twist message
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_x
        cmd_vel_msg.linear.y = linear_y
        cmd_vel_msg.angular.z = angular_z

        # Publish velocity commands
        self.cmd_vel_pub.publish(cmd_vel_msg)

if __name__ == '__main__':
    joy_teleop = JoyTeleop()
    rospy.spin()
