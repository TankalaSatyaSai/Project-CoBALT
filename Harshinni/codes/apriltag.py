#!/usr/bin/env python

#importing the necessary variables 
import numpy as np
import numba #library to optimise numpy inverse calculation
from tf.transformations import euler_from_quaternion #function to calculate yaw angle from quaternion

#calcualting inverse in a more optimised way
@numba.jit(nopython=True)
def inv_matrix(A):
  return np.linalg.inv(A)

def correction_from_april_tag_callback(data,odom_cal):
    
    """Performs Kalman update for position using apriltag information"""
    
    x,y,yaw_apriltag= euler_from_quaternion((data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))
    yaw_apriltag_normalised = (yaw_apriltag +np.pi)%(2*np.pi) - np.pi

    measurement_pose = np.asarray([data.pose.pose.position.x, data.pose.pose.position.y,yaw_apriltag_normalised])
    

    if (abs(measurement_pose[0]) + abs(measurement_pose[1])) <= 1.0:
         return

    #intialising the measurement covariance 
    measurement_covariance = np.zeros((3,3))
    measurement_covariance[0,0] = data.pose.covariance[0]*100
    measurement_covariance[0,1] = data.pose.covariance[1]*10
    measurement_covariance[1,0] = data.pose.covariance[6]*10
    measurement_covariance[1,1] = data.pose.covariance[7]*20
    measurement_covariance[2,1] = 0
    measurement_covariance[0,2] = 0
    measurement_covariance[2,0] = 0
    measurement_covariance[1,2] = 0
    measurement_covariance[2,2] = 200 #a high value is given because apriltag yaw values have an offset

    if measurement_covariance[0,0]<=5:
         return    
    
    covariance_i = odom_cal.covariance.copy()
    covariance_decomposed =  np.linalg.cholesky(covariance_i).T
    measurement_covariance_decomposed = np.linalg.cholesky(measurement_covariance).T
    covariance_i = covariance_decomposed.copy()

    #calculating innovatioon and performing the update
    pose_i = np.array([odom_cal.pose[0],odom_cal.pose[1],odom_cal.pose[2]])
    innovation = measurement_pose - pose_i
    innovation[2] = (innovation[2]+np.pi)%(2*np.pi) - np.pi #normalising the innovation
    innovation_covariance = covariance_i + measurement_covariance_decomposed

    kalman_gain = np.dot(covariance_i, inv_matrix(innovation_covariance))

    pose_correction= np.dot(kalman_gain, innovation)  
    covariance_corrected = np.dot((np.eye(3) - kalman_gain), covariance_i)

    covariance_corrected = np.matmul(covariance_corrected,covariance_corrected.T)

    odom_cal.pose[0]=odom_cal.pose[0]+pose_correction[0]
    odom_cal.pose[1]=odom_cal.pose[1]+pose_correction[1]
    odom_cal.pose[2]=odom_cal.pose[2]+pose_correction[2]
    odom_cal.covariance= covariance_corrected.copy()


#!/usr/bin/env python

import numpy as np
import numba
from tf.transformations import euler_from_quaternion

@numba.jit(nopython=True)
def inv_matrix(A):
    return np.linalg.inv(A)

def calculate_yaw_from_quaternion(quaternion):
    x, y, z, w = quaternion
    yaw = euler_from_quaternion((x, y, z, w))[2]
    return (yaw + np.pi) % (2 * np.pi) - np.pi

def initialize_measurement_covariance(data):
    measurement_covariance = np.zeros((3, 3))
    measurement_covariance[0, 0] = data.pose.covariance[0] * 100
    measurement_covariance[0, 1] = data.pose.covariance[1] * 10
    # ... (similar lines for other elements)
    return measurement_covariance

def normalize_yaw_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def correction_from_april_tag_callback(data, odom_cal):
    x, y, yaw_apriltag = euler_from_quaternion((data.pose.pose.orientation.x,
                                                 data.pose.pose.orientation.y,
                                                 data.pose.pose.orientation.z,
                                                 data.pose.pose.orientation.w))

    yaw_apriltag_normalized = normalize_yaw_angle(yaw_apriltag)
    measurement_pose = np.array([data.pose.pose.position.x, data.pose.pose.position.y, yaw_apriltag_normalized])

    if np.sum(np.abs(measurement_pose[:2])) <= 1.0:
        return

    measurement_covariance = initialize_measurement_covariance(data)

    if measurement_covariance[0, 0] <= 5:
        return

    covariance_i = np.linalg.cholesky(odom_cal.covariance).T
    measurement_covariance_decomposed = np.linalg.cholesky(measurement_covariance).T
    covariance_i = covariance_i.copy()

    pose_i = np.array([odom_cal.pose[0], odom_cal.pose[1], odom_cal.pose[2]])
    innovation = measurement_pose - pose_i
    innovation[2] = normalize_yaw_angle(innovation[2])

    innovation_covariance = covariance_i + measurement_covariance_decomposed

    kalman_gain = np.dot(covariance_i, inv_matrix(innovation_covariance))

    pose_correction = np.dot(kalman_gain, innovation)
    covariance_corrected = np.dot((np.eye(3) - kalman_gain), covariance_i)
    covariance_corrected = np.matmul(covariance_corrected, covariance_corrected.T)

    odom_cal.pose += pose_correction
    odom_cal.covariance = covariance_corrected.copy()
