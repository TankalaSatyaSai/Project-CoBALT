#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Function to get keyboard inputs
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Main function to control the robot using keyboard inputs
def teleop():
    rospy.init_node('teleop_node', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0

    try:
        while not rospy.is_shutdown():
            key = getKey()

            if key == 'w':
                twist.linear.x = 0.5
                twist.angular.z = 0.0
            elif key == 's':
                twist.linear.x = -0.5
                twist.angular.z = 0.0
            elif key == 'a':
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            elif key == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -0.5
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            pub.publish(twist)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        teleop()
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
