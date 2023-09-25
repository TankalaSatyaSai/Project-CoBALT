#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, Spawn
from turtlesim.msg import Pose
import math
import random


target_x = random.randint(0,10)
target_y = random.randint(0,10)




def turtle_spawn(x, y, theta, name):
 srv_name = 'spawn'
 rospy.wait_for_service(srv_name)
 try:
  srv_p = rospy.ServiceProxy(srv_name, Spawn)
  res_m = srv_p(float(x), float(y), float(theta), name)
 except rospy.ServiceException as e:
  print('Turtle Spawn Service failed: ')
  print(e)

def turtle_set_pen(red, green, blue, pen_width, pen_up, name = 'turtle1'):
 rospy.wait_for_service(f'{name}/set_pen')
 try:
  srv_p = rospy.ServiceProxy(f'{name}/set_pen', SetPen)
  res_m = srv_p(int(red), int(green), int(blue), int(pen_width), int(pen_up))
 except rospy.ServiceException as e:
  print('Turtle Set Pen Service failed.')
  print(e)
 
  





def move_turtle1_to_target(pose: Pose):
   
    target_orientation = math.atan2(target_y - pose.y, target_x - pose.x)

    
    angle_diff = target_orientation - pose.theta

    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    elif angle_diff < -math.pi:
        angle_diff += 2 * math.pi


       
      
    if abs(angle_diff) < 0.2:
        del_x = target_x - pose.x
        del_y = target_y - pose.y
        distance = math.sqrt(del_x**2 + del_y**2)

        speed = max(0.1, min(1.0, distance / 10))
        cmd = Twist()
        cmd.linear.x = speed
        pub1.publish(cmd)
    else:
       
          cmd = Twist()
          cmd.angular.z = angle_diff
          pub1.publish(cmd)
   

 




def move_turtle2_to_target(pose: Pose):
    
  
    target_orientation = math.atan2(target_y - pose.y, target_x - pose.x)

    
 
    angle_diff = target_orientation - pose.theta

    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    elif angle_diff < -math.pi:
        angle_diff += 2 * math.pi

    
    if abs(angle_diff) < 0.2:
        del_x = target_x - pose.x
        del_y = target_y - pose.y
        distance = math.sqrt(del_x**2 + del_y**2)

      
        speed = max(0.1, min(1.0, distance / 10))
        cmd = Twist()
        cmd.linear.x = speed
        pub2.publish(cmd)
    else:
        
        cmd = Twist()
        cmd.angular.z = angle_diff
        pub2.publish(cmd)

def move_turtle3_to_target(pose: Pose):
   
    
    target_orientation = math.atan2(target_y - pose.y, target_x - pose.x)

   
    angle_diff = target_orientation - pose.theta

    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    elif angle_diff < -math.pi:
        angle_diff += 2 * math.pi

    if abs(angle_diff) < 0.2:
        del_x = target_x - pose.x
        del_y = target_y - pose.y
        distance = math.sqrt(del_x**2 + del_y**2)

        
        speed = max(0.1, min(1.0, distance / 10))
        cmd = Twist()
        cmd.linear.x = speed
        pub3.publish(cmd)
    else:
        if pose.x != target_x and pose.y != target_y:
           
          cmd = Twist()
          cmd.angular.z = angle_diff
          pub3.publish(cmd)
        else:
           cmd = Twist()
           cmd.linear = 0.0
           cmd.angular = 0.0



if __name__ == '__main__':
    rospy.init_node('turtle_move')
     
    x1 = random.randint(0, 10)
    x2 = random.randint(0, 10)
    y1 = random.randint(0, 10)
    y2 = random.randint(0, 10)
    turtle_spawn(x1, y1, 10, 'turtle2')
    turtle_spawn(x2, y2, 90, 'turtle3')

    turtle_set_pen(0, 255, 0, 5, 0, 'turtle1')
    turtle_set_pen(255, 0, 0, 5, 0, 'turtle2')
    turtle_set_pen(200, 0, 255, 5, 0, 'turtle3')
    sub1 = rospy.Subscriber('/turtle1/pose', Pose, callback=move_turtle1_to_target)  
    sub2 = rospy.Subscriber('/turtle2/pose', Pose, callback=move_turtle2_to_target)
    sub3 = rospy.Subscriber('/turtle3/pose', Pose, callback=move_turtle3_to_target)
    pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    pub3 = rospy.Publisher('/turtle3/cmd_vel', Twist, queue_size=10)
    rospy.spin()
