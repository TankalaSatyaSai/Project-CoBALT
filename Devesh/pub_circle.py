#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist




    
def publisher_node():
    rospy.init_node('move_turtlebot')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    
    rate = rospy.Rate(10)

    
    while not rospy.is_shutdown():     
      twist = Twist()
      twist.angular.z = 0.5
      twist.linear.x = 0.5
      pub.publish(twist)
          
      rate.sleep()


    






if __name__ == '__main__':
    try:
        publisher_node()
        
    except rospy.ROSInternalException:
        pass
