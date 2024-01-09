#importing the necessary variables 
import numpy as np #numerical python library

def callback_yaw_imu(msg, odom_cal):  

    "Implements the Kalman update based on IMU value"

    if odom_cal.imu_angle_offset!=None:
    
        #updating the yaw angle as obtained from imu 
        odom_cal.imu_yaw = ((msg.angular.z + 180.0) % 360.0) - 180.0 #Constrain between -180 to 180
        odom_cal.imu_yaw = odom_cal.imu_yaw*(np.pi/180.0) #convert to radian 
        imu_updated_yaw = odom_cal.imu_yaw - odom_cal.imu_angle_offset #substracting the offset
        imu_updated_yaw = (imu_updated_yaw+np.pi)%(2*np.pi)-np.pi #Constrain between -180 to 180

        #storing the current yaw and varaince constrained between -pi to pi
        current_yaw = ((odom_cal.pose[2] + np.pi)%(2*np.pi)) - np.pi 
        current_variance = odom_cal.covariance[2][2]
        innovation = ((imu_updated_yaw- current_yaw) +np.pi)%(2*np.pi)-np.pi
        
        #appyling kalman filter based correction on the yaw angle mean and variance    
        odom_cal.pose[2] = current_yaw + (current_variance/(current_variance + odom_cal.variance_imu))*(innovation)
        odom_cal.covariance[2][2] = current_variance - (current_variance/(current_variance + odom_cal.variance_imu))*(current_variance)

        #renormalising the covariance matrix such that it stays positive semidefinite
        odom_cal.covariance[1,2] = odom_cal.covariance[1,2]*(np.sqrt(odom_cal.covariance[2,2]))/(np.sqrt(current_variance))
        odom_cal.covariance[2,0] = odom_cal.covariance[2,0]*(np.sqrt(odom_cal.covariance[2,2]))/(np.sqrt(current_variance))
        odom_cal.covariance[2,1] = odom_cal.covariance[1,2]
        odom_cal.covariance[0,2] = odom_cal.covariance[2,0]

    else:
        
        odom_cal.imu_yaw = ((msg.angular.z + 180.0) % 360.0) - 180.0 #Constrain between -180 to 180
        odom_cal.imu_yaw = odom_cal.imu_yaw*(np.pi/180.0)
        #odom_cal.imu_angle_offset= odom_cal.imu_yaw
        odom_cal.imu_angle_offset = (-180)*np.pi/180 #hard coding the offset value