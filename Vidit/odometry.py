#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

def imu_callback(data):
    gyro_angular_velocity = data.angular_velocity.z
    acceleration_angle = np.arctan2(data.linear_acceleration.y, data.linear_acceleration.x)

    theta = alpha * (theta_gyro + theta) + (1 - alpha) * acceleration_angle

def encoder_callback(data):
    global x, y, theta  # Declare x, y, and theta as global to update their values

    encoder_counts_per_second = data.position
    counts_to_radians = 2 * np.pi / 360.0
    wheel_radius = 0.05 

    wheel_velocities = [count * counts_to_radians for count in encoder_counts_per_second]
    v1, v2, v3, v4 = [velocity * wheel_radius for velocity in wheel_velocities]

    vx = (1 / 4) * (v1 + v2 + v3 + v4)
    vy = (1 / 4) * (v1 - v2 - v3 + v4)
    omega = (1 / (4 * wheel_base)) * (v1 - v2 + v3 - v4)

    x += vx * np.cos(theta) * time_elapsed_encoder
    y += vy * np.sin(theta) * time_elapsed_encoder

    current_position_msg = Twist()
    current_position_msg.linear.x = x
    current_position_msg.linear.y = y
    current_position_msg.angular.z = theta
    position_publisher.publish(current_position_msg)

if __name__ == '__main__':
   
    rospy.init_node('robot_position_tracker')
  
    alpha = 0.75

    x = 0.0
    y = 0.0
    theta = 0.0

    time_elapsed_encoder = 0.1  

    imu_subscriber = rospy.Subscriber('/imu', Imu, imu_callback)
    encoder_subscriber = rospy.Subscriber('/joint_states', JointState, encoder_callback)

    position_publisher = rospy.Publisher('/current_position', Twist, queue_size=10)

    rospy.spin()

