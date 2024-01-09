#!/usr/bin/env python
 
import numpy as np  
from math import sin, cos, pi #importing the necessary library 
import rospy #python library for ROS
import tf #importing tf transforms for doing various transformations
from geometry_msgs.msg import PoseWithCovarianceStamped #importig message for publish
from geometry_msgs.msg import Twist #using Twist message for imu topic
from imu_odom.srv import odom_reset,odom_resetResponse #importing the nescessary service request and response messages  
from imu_odom.msg import ticks #custom message for encoder ticks
from imu_odom.msg import FeatureAsArray #using a predefined message for line features
import param_def #global parameters file 
import imu_def #importing the file containing the functions related to imu 

class OdometryCalculation:

    # Variables storing the previous left and right tick for each wheel
    prev_left_tick_wheel_1 = 0
    prev_right_tick_wheel_1 = 0
    prev_left_tick_wheel_2 = 0
    prev_right_tick_wheel_2 = 0
    prev_left_tick_wheel_3 = 0
    prev_right_tick_wheel_3 = 0
    prev_left_tick_wheel_4 = 0
    prev_right_tick_wheel_4 = 0

    # Variables storing the current left and right tick for each wheel
    left_tick_wheel_1 = 0
    right_tick_wheel_1 = 0
    left_tick_wheel_2 = 0
    right_tick_wheel_2 = 0
    left_tick_wheel_3 = 0
    right_tick_wheel_3 = 0
    left_tick_wheel_4 = 0
    right_tick_wheel_4 = 0

    # Variables to keep track of IMU yaw angle for the combined robot
    imu_yaw = None
    variance_imu = (1.0 * np.pi / 180.0) ** 2
    imu_angle_offset = None

    # Pose and covariance variables for each wheel
    pose_wheel_1 = np.array([0.0, 0.0, 0.0])
    pose_wheel_2 = np.array([0.0, 0.0, 0.0])
    pose_wheel_3 = np.array([0.0, 0.0, 0.0])
    pose_wheel_4 = np.array([0.0, 0.0, 0.0])

    covariance_combined = np.diag([0.0, 0.0, 0.0])
    covariance_wheel_1 = np.diag([0.0, 0.0, 0.0])
    covariance_wheel_2 = np.diag([0.0, 0.0, 0.0])
    covariance_wheel_3 = np.diag([0.0, 0.0, 0.0])
    covariance_wheel_4 = np.diag([0.0, 0.0, 0.0])


    def get_system_covariance(self, wheel_movements):
        """
        Calculates the increase in covariance due to previous state.
        Takes into account four mecanum wheels.
        """

        # Constants
        width_robot = param_def.width_robot  # Robot width
        delta_theta_factor = param_def.delta_theta_factor  # Factor for delta_theta

        # Extract wheel movements for each wheel
        wheel1, wheel2, wheel3, wheel4 = wheel_movements

        # Calculate average wheel movement
        delta_d = (wheel1 + wheel2 + wheel3 + wheel4) / 4
        delta_theta = delta_theta_factor * (wheel2 - wheel1 - wheel4 + wheel3) / width_robot

        # Calculate the Jacobian of the states with respect to the previous state
        F_p = np.array([
            [1, 0, -delta_d * np.sin(self.pose[2] + delta_theta / 2)],
            [0, 1, delta_d * np.cos(self.pose[2] + delta_theta / 2)],
            [0, 0, 1]
        ])

        return F_p

    def get_motion_covariance(self, wheel_movements):
        """
        Calculates the increase in covariance due to control inputs.
        Takes into account four mecanum wheels.
        """

        # Constants
        width_robot = param_def.width_robot  # Robot width

        # Extract wheel movements for each wheel
        wheel1, wheel2, wheel3, wheel4 = wheel_movements

        # Calculate average wheel movements
        delta_d = (wheel1 + wheel2 + wheel3 + wheel4) / 4
        delta_theta = (wheel2 - wheel1 - wheel4 + wheel3) / width_robot

        # Calculate the components of the Jacobian of the state with respect to the control inputs
        f11 = (1 / 2) * (np.cos(self.pose[2] + delta_theta / 2)) - (delta_d / (2 * width_robot)) * np.sin(self.pose[2] + delta_theta / 2)
        f12 = (1 / 2) * (np.cos(self.pose[2] + delta_theta / 2)) + (delta_d / (2 * width_robot)) * np.sin(self.pose[2] + delta_theta / 2)

        f21 = (1 / 2) * (np.sin(self.pose[2] + delta_theta / 2)) + (delta_d / (2 * width_robot)) * np.cos(self.pose[2] + delta_theta / 2)
        f22 = (1 / 2) * (np.sin(self.pose[2] + delta_theta / 2)) - (delta_d / (2 * width_robot)) * np.cos(self.pose[2] + delta_theta / 2)

        f31 = 1 / width_robot
        f32 = -1 / width_robot

        F_u = np.array([
            [f11, f12],
            [f21, f22],
            [f31, f32]
        ])

        return F_u


    def get_covariance(self):
        """Calculates the covariance of the new state for a robot with four mecanum wheels."""
        
        # Calculate tick increment for each wheel
        tick_difference = [0, 0, 0, 0]  # Initializing the tick difference array for the four wheels
        tick_difference[0] = self.wheel1_tick - self.prev_wheel1_tick  # Assigning the tick difference for wheel 1
        tick_difference[1] = self.wheel2_tick - self.prev_wheel2_tick  # Assigning the tick difference for wheel 2
        tick_difference[2] = self.wheel3_tick - self.prev_wheel3_tick  # Assigning the tick difference for wheel 3
        tick_difference[3] = self.wheel4_tick - self.prev_wheel4_tick  # Assigning the tick difference for wheel 4

        # Extracting the control input for each wheel
        wheel_movements = [tick * param_def.ticks_to_millimeter for tick in tick_difference]

        if (wheel != 0 for wheel in wheel_movements):
            # Extracting motion and turn factors
            alpha_1 = param_def.control_motion_factor
            alpha_2 = param_def.control_turn_factor

            # Calculate absolute distances moved by each wheel
            abs_wheel_movements = [abs(movement) for movement in wheel_movements]

            # Defining the covariances associated with each wheel's control
            sigma_wheels = [
                (alpha_1 * abs_wheel + alpha_2 * abs(abs_wheel - sum(abs_wheel_movements) - abs_wheel))
                for abs_wheel in abs_wheel_movements
            ]

            control_covariance = np.diag(sigma_wheels)  # Forming a diagonal matrix using the control covariance

            # Getting the covariance involved due to the covariance in the previous state
            F_p = self.get_system_covariance(wheel_movements)

            # Getting the covariance involved due to the error in the control input
            F_u = self.get_motion_covariance(wheel_movements)

            # Calculating the final covariance
            self.covariance = np.dot(F_p, np.dot(self.covariance, F_p.T)) + np.dot(F_u, np.dot(control_covariance, F_u.T))

    def odometry_update(self):

        """Calculates the new state based on the old state and change in encoder ticks for a robot with four mecanum wheels."""
        
        # Calculate tick increment for each wheel
        tick_difference = [0, 0, 0, 0]  # Initializing the tick difference array for the four wheels
        tick_difference[0] = self.wheel1_tick - self.prev_wheel1_tick  # Assigning the tick difference for wheel 1
        tick_difference[1] = self.wheel2_tick - self.prev_wheel2_tick  # Assigning the tick difference for wheel 2
        tick_difference[2] = self.wheel3_tick - self.prev_wheel3_tick  # Assigning the tick difference for wheel 3
        tick_difference[3] = self.wheel4_tick - self.prev_wheel4_tick  # Assigning the tick difference for wheel 4

        # Update the previous tick values in the class
        self.prev_wheel1_tick = self.wheel1_tick
        self.prev_wheel2_tick = self.wheel2_tick
        self.prev_wheel3_tick = self.wheel3_tick
        self.prev_wheel4_tick = self.wheel4_tick

        # Variables to store new pose and change in orientation
        x = 0.0
        y = 0.0
        theta = 0.0
        delta_theta = 0.0

        # Calculate robot motion based on tick increment
        # First case: robot travels in a straight line
        if all(tick == tick_difference[0] for tick in tick_difference):
            theta = self.pose[2]  # Get the previous orientation
            x = self.pose[0] + tick_difference[0] * param_def.ticks_to_millimeter * cos(theta)  # Update x
            y = self.pose[1] + tick_difference[1] * param_def.ticks_to_millimeter * sin(theta)  # Update y

        # Second case: in case of a curve
        else:
            theta = self.pose[2]  # Get the previous orientation
            x = self.pose[0] - param_def.odometry_offset * sin(theta)
            y = self.pose[1] - param_def.odometry_offset * cos(theta)

            # Change in orientation and radius of curvature of the turn
            delta_theta = param_def.ticks_to_millimeter * (tick_difference[1] - tick_difference[0]) / param_def.width_robo  # Change in orientation
            R = param_def.ticks_to_millimeter * tick_difference[0] / delta_theta  # Radius of curvature

            # Calculating the center of curvature
            centerx = x - (R + param_def.width_robo / 2) * sin(theta)  # x coordinate
            centery = y + (R + param_def.width_robo / 2) * cos(theta)  # y coordinate

            # Updating the x and using the newly calculated theta value
            theta = theta + delta_theta
            x = centerx + ((R + param_def.width_robo / 2) * sin(theta)) + (param_def.odometry_offset * sin(theta))  # Calculating the new x
            y = centery - ((R + param_def.width_robo / 2) * cos(theta)) + (param_def.odometry_offset * cos(theta))  # Calculating the new y

        # Update x and y position
        self.pose[0] = x
        self.pose[1] = y
        self.pose[2] = theta
        

        def publish_pose(self, publisher):
            """Publishes the resulting pose for a 4-mecanum-wheeled robot"""

            # Extract pose variables for each wheel (replace with actual values)
            X1 = self.pose_wheel_1[0]  # X position for wheel 1
            Y1 = self.pose_wheel_1[1]  # Y position for wheel 1
            theta1 = self.pose_wheel_1[2]  # Orientation for wheel 1

            X2 = self.pose_wheel_2[0]  # X position for wheel 2
            Y2 = self.pose_wheel_2[1]  # Y position for wheel 2
            theta2 = self.pose_wheel_2[2]  # Orientation for wheel 2

            X3 = self.pose_wheel_3[0]  # X position for wheel 3
            Y3 = self.pose_wheel_3[1]  # Y position for wheel 3
            theta3 = self.pose_wheel_3[2]  # Orientation for wheel 3

            X4 = self.pose_wheel_4[0]  # X position for wheel 4
            Y4 = self.pose_wheel_4[1]  # Y position for wheel 4
            theta4 = self.pose_wheel_4[2]  # Orientation for wheel 4

            # Combine the pose variables for all wheels (modify as needed)
            X_combined = (X1 + X2 + X3 + X4) / 4.0
            Y_combined = (Y1 + Y2 + Y3 + Y4) / 4.0
            theta_combined = (theta1 + theta2 + theta3 + theta4) / 4.0

            # Calculate the quaternion for the combined orientation (modify as needed)
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta_combined)

            odom = PoseWithCovarianceStamped()
            odom.pose.pose.position.x = X_combined
            odom.pose.pose.position.y = Y_combined
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation.x = odom_quat[0]
            odom.pose.pose.orientation.y = odom_quat[1]
            odom.pose.pose.orientation.z = odom_quat[2]
            odom.pose.pose.orientation.w = odom_quat[3]

            # Set the covariance matrix values (modify as needed)
            covariance_values = [
                self.covariance_combined[0][0], self.covariance_combined[0][1], self.covariance_combined[0][2], 0, 0, 0,
                self.covariance_combined[1][0], self.covariance_combined[1][1], self.covariance_combined[1][2], 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, self.covariance_combined[2][0], self.covariance_combined[2][1],
            ]

            odom.pose.covariance = covariance_values

            # Publish the message
            publisher.publish(odom)
            return

    def callback_encoder_tick(self, msg):
        self.left_tick_wheel_1 = msg.left_tick_wheel_1  # Update the left_tick for wheel 1
        self.right_tick_wheel_1 = msg.right_tick_wheel_1  # Update the right tick for wheel 1
        self.left_tick_wheel_2 = msg.left_tick_wheel_2  # Update the left_tick for wheel 2
        self.right_tick_wheel_2 = msg.right_tick_wheel_2  # Update the right tick for wheel 2
        self.left_tick_wheel_3 = msg.left_tick_wheel_3  # Update the left_tick for wheel 3
        self.right_tick_wheel_3 = msg.right_tick_wheel_3  # Update the right tick for wheel 3
        self.left_tick_wheel_4 = msg.left_tick_wheel_4  # Update the left_tick for wheel 4
        self.right_tick_wheel_4 = msg.right_tick_wheel_4  # Update the right tick for wheel 4

    def set_odometry(self, req):
        """Resets the pose as the input of the service for a 4-mecanum-wheeled robot"""
        self.pose_wheel_1 = np.array([req.x, req.y, req.theta])  # Update the pose for wheel 1
        self.pose_wheel_2 = np.array([req.x, req.y, req.theta])  # Update the pose for wheel 2
        self.pose_wheel_3 = np.array([req.x, req.y, req.theta])  # Update the pose for wheel 3
        self.pose_wheel_4 = np.array([req.x, req.y, req.theta])  # Update the pose for wheel 4
        self.covariance_combined = np.diag([10.0**2, 10.0**2, (5.0 / 180.0 * pi) ** 2])  # Setting the initial covariance for the combined robot
        # You should add similar lines for the other three wheels

        return odom_resetResponse(True)  # Setting the response variable in the srv file to be true
    
def main():

    #Class object for odometry calculation
    odom_cal = OdometryCalculation()

    #Creating a publisher topic and a node named localisation 
    pub_node = rospy.Publisher("rand", queue_size=1)
   
    #Different subscribers involved 
    rospy.Subscriber('/Encoder_Ticks', ticks, odom_cal.callback_encoder_tick) #subscriber to subscribe to the left motor_ticks
    rospy.Subscriber('/pozyx_position', Twist, imu_def.callback_yaw_imu,(odom_cal)) #subscriber to subscribe to imu topic
    
        #Intialising the odom_node
    rospy.init_node('fusion', anonymous=True)

    #Creating a service called reset_odometry to set initial pose to odometry with large covariance (can be used to simulate kidnapped robot)
    service_reset_odometry = rospy.Service('reset_odometry', odom_reset, odom_cal.set_odometry)

    #Initialising the rate variable to an appropriate rate 
    rate = rospy.Rate(10) # 10hz

    #begining pose estimation
    print("Starting Localisation")

    #loop for rospy
    while not rospy.is_shutdown():

        odom_cal.get_covariance() #getting the new covariance asssicated with the previous pose using new control

        odom_cal.odometry_update() #updating pase using encoder update 
        
        odom_cal.publish_pose(pub_node) #publish the new pose with covariance

        rate.sleep() #stoping the loop to maintian rate 
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: #exception for ROSInteruppt
        pass

