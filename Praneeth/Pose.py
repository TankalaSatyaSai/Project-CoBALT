#!/usr/bin/env python3

#Importing the nescessary libraries 
import numpy as np #importing thr numericla python library
from math import sin, cos, pi #importing the necessary library 
import rospy #python library for ROS
import tf #importing tf transforms for doing various transformations
from geometry_msgs.msg import PoseWithCovarianceStamped #importig message for publish
# from localisation.srv import odom_reset,odom_resetResponse #importing the nescessary service request and response messages  
# from localisation.msg import ticks #custom message for encoder ticks
from geometry_msgs.msg import Twist #using Twist message for imu topic
# import param_set #global parameters file 
# import imu_functions #importing the file containing the functions related to imu 
# import april_tag_functions #importing the file containing functions for apriltag based localisation
# import lidar_functions #importing the file containing functions for apriltag based localisation
# from localisation.msg import FeatureAsArray#using a predefined message for line features
from std_msgs.msg import Int64  

#class to keep track of the ticks which arise from the motor encoders
class Odometry_calculation:

    #variables storing the previous left and right tick 
    prev_front_left_tick=Int64(data=0)
    prev_front_right_tick=Int64(data=0)
    prev_rear_left_tick=Int64(data=0)
    prev_rear_right_tick=Int64(data=0)

    #variables stroing the current left and right tick
    front_left_tick=Int64(data=0)
    front_right_tick=Int64(data=0)
    rear_left_tick=Int64(data=0)
    rear_right_tick=Int64(data=0)

    #variable to keep track of IMU yaw angle
    # imu_yaw=None
    # variance_imu= (1.0*np.pi/180.0)**2
    # imu_angle_offset=None

    pose = np.array([0.0, 0.0, 0.0]) #Variable to store pose of the robot 

    def odometry_update(self):

        """Calcualtes the new state based on old state and change in encoder ticks"""
    
        #Calculate tick increment 
        # tick_difference=[0,0,0,0] #intialisng the tick difference array which is the control input
        tick_difference=np.array([[0],
                          [0],
                          [0],
                          [0]])
        tick_difference[0][0] = self.front_left_tick.data - self.prev_front_left_tick.data #assigning the left tick difference 
        tick_difference[1][0] = self.front_right_tick.data - self.prev_front_right_tick.data #assigning the right tick difference 
        tick_difference[2][0] = self.rear_left_tick.data - self.prev_rear_left_tick.data #assigning the left tick difference 
        tick_difference[3][0] = self.rear_right_tick.data - self.prev_rear_right_tick.data #assigning the right tick difference 
        
        self.prev_front_left_tick = self.front_left_tick #updating the left tick variable in the class
        self.prev_front_right_tick = self.front_right_tick #updating the right tick variable in the class
        self.prev_rear_left_tick = self.rear_left_tick
        self.prev_rear_right_tick = self.rear_right_tick 

        a = 0.76  #wheel_radius
        d = 0.270 #robot_width
        l = 0.215 #separation_bet_wheels/2
        
    
        conversion_matrix = np.array([[1, 1, 1, 1],
                                      [-1, 1, 1, -1],
                                      [-1/(l+d), 1/(l+d), -1/(l+d), 1/(l+d)]])
        
        delta = (2*np.pi/7) * (a/4) * np.dot(conversion_matrix , tick_difference)

        self.pose[0] = self.pose[0] + delta[0]/425
        self.pose[1] = self.pose[1] + delta[1]/425
        self.pose[2] = self.pose[2] + delta[2]/500

        return

    #publisher for new data
    def publish_pose(self, publisher):

        """Publishes the resulting pose"""

        #assiging each pose vaiable into separate X Y and theta
        X=self.pose[0]
        Y=self.pose[1]
        theta=self.pose[2]

        #getting the quaternion for tf tranform concerning odometry and pozyx
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)        

        #creating a instance of the custom odometry message 
        odom = PoseWithCovarianceStamped()
        odom.pose.pose.position.x = X
        odom.pose.pose.position.y = Y
        odom.pose.pose.position.z = np.rad2deg(theta)
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        
        # publish the message
        publisher.publish(odom) #publishing the odometry message 

        return
    
    #call back function to update ticks    
    
    def callback_encoder_tick_1(self, msg):

        """Updates the encoder data"""

        self.front_left_tick = msg #updating the left_tick
        # self.front_right_tick = msg.front_right_tick#updating the right tick
        # self.rear_left_tick = msg.rear_left_tick#updating the right tick
        # self.rear_right_tick = msg.rear_right_tick#updating the right tick

        return
    def callback_encoder_tick_2(self, msg):

        """Updates the encoder data"""

        # self.front_left_tick = msg.front_left_tick #updating the left_tick
        self.front_right_tick = msg #updating the right tick
        # self.rear_left_tick = msg.rear_left_tick#updating the right tick
        # self.rear_right_tick = msg.rear_right_tick#updating the right tick

        return
    def callback_encoder_tick_3(self, msg):

        """Updates the encoder data"""

        # self.front_left_tick = msg.front_left_tick #updating the left_tick
        # self.front_right_tick = msg.front_right_tick#updating the right tick
        self.rear_left_tick = msg #updating the right tick
        # self.rear_right_tick = msg.rear_right_tick#updating the right tick

        return
    def callback_encoder_tick_4(self, msg):

        """Updates the encoder data"""

        # self.front_left_tick = msg.front_left_tick #updating the left_tick
        # self.front_right_tick = msg.front_right_tick#updating the right tick
        # self.rear_left_tick = msg.rear_left_tick#updating the right tick
        self.rear_right_tick = msg #updating the right tick

        return
    #call back function for odom reset service
    # def set_odometry(self, req):

    #     """Resets the pose as the input of the service"""

    #     self.pose=np.array([req.x,req.y,req.theta]) #getting the required variable x, y, theta .

    #     self.covariance = np.diag([10.0**2, 10.0**2, (5.0 / 180.0 * pi) ** 2]) #setting the initial covarince 

    #     return odom_resetResponse(True) #setting the response variable in the srv file to be true

    def set_odometry(self):

        """Resets the pose as the input of the service"""

        self.pose=np.array([0,0,0])

def main():

    #Class object for odometry calculation
    odom_cal = Odometry_calculation()

    

    #Creating a publisher topic and a node named localisation 
    odom_pub = rospy.Publisher("pose_ekf", PoseWithCovarianceStamped, queue_size=1)
   
    #Different subscribers involved 
    rospy.Subscriber('/front_left_ticks', Int64, odom_cal.callback_encoder_tick_1) #subscriber to subscribe to the f_l motor_ticks
    rospy.Subscriber('/front_right_ticks', Int64, odom_cal.callback_encoder_tick_2) #subscriber to subscribe to the f_r motor_ticks
    rospy.Subscriber('/rear_left_ticks', Int64, odom_cal.callback_encoder_tick_3) #subscriber to subscribe to the r_l motor_ticks
    rospy.Subscriber('/rear_right_ticks', Int64, odom_cal.callback_encoder_tick_4) #subscriber to subscribe to the r_r motor_ticks
    


    #Intialising the odom_node
    rospy.init_node('localisation', anonymous=True)

    #Creating a service called reset_odometry to set initial pose to odometry with large covariance (can be used to simulate kidnapped robot)
    # service_reset_odometry = rospy.Service('reset_odometry', odom_reset, odom_cal.set_odometry)

    odom_cal.set_odometry

    #Initialising the rate variable to an appropriate rate 
    rate = rospy.Rate(10) # 10hz

    #begining pose estimation
    print("Starting Localisation")

    #loop for rospy
    while not rospy.is_shutdown():

        # odom_cal.get_covariance() #getting the new covariance asssicated with the previous pose using new control

        odom_cal.odometry_update() #updating pase using encoder update 
        
        odom_cal.publish_pose(odom_pub) #publish the new pose with covariance

        rate.sleep() #stoping the loop to maintian rate 
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: #exception for ROSInteruppt
        pass
