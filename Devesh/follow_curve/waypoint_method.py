#!/usr/bin/env python3

from waypoints import waypoint_x, waypoint_y
import rospy
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry
import csv
a , b = 0, 0
target_x = 0
target_y = 0
prev_err_x = 0
prev_err_y = 0


def main_fun():
    global target_x, target_y,a,b
    rospy.init_node('follow_the_curve')
    rospy.Subscriber('/odom', Odometry, callback=calculate_vel)
    rate = rospy.Rate(10)

   
    rospy.spin()




def check_for_vel(current_x,current_y, X, Y):

    if X > current_x:
        x_vel = 0.2
    if X < current_x:
        x_vel = -0.2
    if X == current_x:
        x_vel = 0.0

    if Y > current_y:
        y_vel = 0.2
    if Y < current_y:
        y_vel = -0.2
    if Y == current_y:
        y_vel = 0.0

    return x_vel, y_vel

    

# def calculate_vel(msg):
#     global target_x, target_y, a, b
#     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     cmd_vel = Twist()
    
#     current_x = msg.pose.pose.position.x
#     current_y = msg.pose.pose.position.y

#     x_vel, y_vel = check_for_vel(current_x, current_y, target_x, target_y)

#     if x_vel == 0.0 and y_vel == 0.0:
#         target_x = waypoint_x[a]
#         target_y = waypoint_y[b]
#         a = a+1
#         b = b+1


#     cmd_vel.linear.x = x_vel
#     cmd_vel.linear.y = y_vel

#     pub.publish(cmd_vel)

#     rospy.loginfo(str(cmd_vel.linear.x) + str(cmd_vel.linear.y))

def calculate_vel(msg):
    global target_x, target_y, a, b, prev_err_x, prev_err_y

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd_vel = Twist()
    
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    x_diff = target_x - current_x
    y_diff = target_y - current_y

    # Set linear velocities based on the difference
    cmd_vel.linear.x = 0.2 + 0.5*x_diff + 0.2*(x_diff-prev_err_x) if x_diff > 0 else -0.2 + 0.5*x_diff + 0.2*(x_diff-prev_err_x) if x_diff < 0 else 0.0
    cmd_vel.linear.y = 0.2 + 0.5*y_diff + 0.2*(y_diff-prev_err_y) if y_diff > 0 else -0.2 + 0.5*y_diff + 0.2*(y_diff-prev_err_y) if y_diff < 0 else 0.0

    # Check if the robot is close to the target waypoint
    distance_threshold = 0.05  # Adjust this value as needed
    if abs(x_diff) < distance_threshold and abs(y_diff) < distance_threshold:
        if a < len(waypoint_x) and b < len(waypoint_y):
            target_x = waypoint_x[a]
            target_y = waypoint_y[b]
            a += 1
            b += 1

    pub.publish(cmd_vel)

    file_name = 'logfile/data_log1.csv'
    

    


    # rospy.loginfo("target_x: " + str(target_x) + " target_y: " +  str(target_y) + " cmd_vel_x : " + str(cmd_vel.linear.x) + " cmd_vel_y: " + str(cmd_vel.linear.y) + " err_x : " + str(current_x - target_x) + " err_y : " + str(current_y - target_y))
    # Log data to the CSV file
    with open(file_name, 'a', newline='') as csv_file:  # 'a' for append mode
        csv_writer = csv.writer(csv_file)

        # Get the data to log (replace with your actual data)
        timestamp = rospy.get_time()
        data_field_3 = target_x
        data_field_5 = target_y
        data_field_1 = cmd_vel.linear.x
        data_field_2 = cmd_vel.linear.y
        data_field_4 = current_x
        data_field_6 = current_y

        # Log the data to the CSV file
        csv_writer.writerow([timestamp, data_field_1, data_field_2, data_field_3, data_field_4, data_field_5, data_field_6])



    



if __name__=='__main__':
    try:
        main_fun()
    except rospy.ROSInterruptException:
        pass



