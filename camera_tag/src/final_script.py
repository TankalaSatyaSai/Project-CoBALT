#!/usr/bin/env python

import rospy
import rospkg
import tf
from tf.transformations import quaternion_matrix, quaternion_from_matrix, compose_matrix, translation_from_matrix, inverse_matrix, identity_matrix, euler_from_quaternion
import numpy as np
import yaml
import io
from geometry_msgs.msg import TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped

class FindRobot():
	def __init__(self):
		self.robotpose = None
		self.position_camera = None
		self.quaternion_camera = None
		self.published = False

	@staticmethod	
	def get_tag_tf(msg):		# Transform from tag to camera
		current = rospy.Time.now()
		T = {}
		# listener = tf.TransformListener()
		t = tf.Transformer(True, rospy.Duration(10.0))
		for det in msg.detections:
			tag_str = 'tag_'+str(det.id[0])
			translation = det.pose.pose.pose.position
			rotation = det.pose.pose.pose.orientation
			if (translation.x**2 + translation.z**2)<=3.0**2:
				# rospy.loginfo(abs(translation.x) + abs(translation.y))
				T[det.id[0]] = identity_matrix()
				T_quat = quaternion_matrix([rotation.x,rotation.y,rotation.z,rotation.w])
				r_trans = T_quat[:3,:3].Tma
				p = np.array([translation.x,translation.y,translation.z])
				T[det.id[0]][:3,:3] = r_trans
				T[det.id[0]][:3,3] = -np.matmul(r_trans,p)
		return T
	
	@staticmethod
	def get_tag_world():		# Transform from world to tag
		tags_dict = {}
		rospack = rospkg.RosPack()
		base_path = rospack.get_path('camera_tag')
		with open(base_path+"/config/tag_positions", 'r') as stream:
			tags = yaml.safe_load(stream)
			tags = tags['tag_pos']
			for i in tags:
				tags_dict[i['id']] = compose_matrix(angles=(i['ex'],i['ey'],i['ez']),translate=(i['x'],i['y'],i['z']))
		return tags_dict
	
	@staticmethod
	def get_camera_tf(TW_dict,CT_dict):	# Transform from world to camera = T_world to tag * T_tag to camera
		tlist = []
		for key in CT_dict.keys():
			tlist.append(np.matmul(TW_dict[key],CT_dict[key]))
		T = sum(tlist)/len(tlist)
		# quat = tf.transformations.quaternion_from_matrix(T)
		# pos = tuple(T[0:3,3])
		return T
	
	@staticmethod
	def get_cov(X):
		cov = np.zeros((3,3))
		parameters = np.array([[[ 1.54994968e+00, -6.93276439e-01,  4.12218279e+00,  7.32972045e-04, 6.23639204e-05],\
						[-1.40360525e-04,  1.75461898e-05, -1.50194389e-05, -4.78351330e+02, 4.78448922e+02],\
						[-1.07300016e-04,  1.47483180e-05, -3.48502305e-05, -1.58087857e+02, 1.58112861e+02]],\
					  [[-1.40360525e-04,  1.75461898e-05, -1.50194389e-05, -4.78351330e+02, 4.78448922e+02],\
						[ 4.49445021e+00, -1.14846351e+01,  2.57681453e+01,  5.13880989e-02, 8.96429090e-07],\
						[ 3.83517793e+00, -9.94496202e+00,  2.20799842e+01,  2.17761351e-02, 2.25936553e-06]],\
					  [[-1.07300016e-04,  1.47483180e-05, -3.48502305e-05, -1.58087857e+02, 1.58112861e+02],\
						[ 3.83517793e+00, -9.94496202e+00,  2.20799842e+01,  2.17761351e-02, 2.25936553e-06],\
						[ 3.25659735e+00, -8.34205799e+00,  1.81003141e+01,  1.13448549e-02, 4.22279486e-06]]])
		
		for i in range(3):
			for j in range(3):
				cov[i,j] = parameters[i,j,4]*np.exp(np.matmul(X.T,parameters[i,j,:3])) + parameters[i,j,3]
		new_cov = np.zeros((36,))
		new_cov[0] = cov[0,0]*1e6
		new_cov[1] = cov[0,1]*1e6
		new_cov[5] = cov[0,2]*1e3
		new_cov[6] = cov[1,0]*1e6
		new_cov[7] = cov[1,1]*1e6
		new_cov[11] = cov[1,2]*1e3
		new_cov[30] = cov[2,0]*1e3
		new_cov[31] = cov[2,1]*1e3
		new_cov[35] = cov[2,2]
		return new_cov


	def send_camera_pose(self, msg):
		TW = self.get_tag_world()
		T_rc = tf.transformations.euler_matrix(0, -1.5708, 1.5708)  # camera orientation w.r.t robot, assuming no translation between robot and camera

		if msg.detections:  # Check if detections array is not empty
			self.published = True
			self.robotpose = PoseWithCovarianceStamped()
			for detection in msg.detections:
				tag_id = detection.id[0]
				CT = self.get_tag_tf(detection)  # Get transformation from tag to camera for the current detection
				if CT:
					if tag_id in TW:  # Check if tag id exists in the world transformation dictionary
						T_cw = np.matmul(TW[tag_id], CT)
						T_rw = np.matmul(T_cw, T_rc)
						self.position_camera = translation_from_matrix(T_cw)
						self.position_camera[0] = round(self.position_camera[0] * 10) * 100
						self.position_camera[1] = round(self.position_camera[1] * 10) * 100
						self.position_camera[2] = 0.0
						quaternion_robot = quaternion_from_matrix(T_rw)
						self.quaternion_camera /= np.linalg.norm(self.quaternion_camera)
						self.robotpose.pose.pose.position.x = self.position_camera[0]
						self.robotpose.pose.pose.position.y = self.position_camera[1]
						self.robotpose.pose.pose.position.z = self.position_camera[2]
						self.robotpose.pose.pose.orientation.x = quaternion_robot[0]
						self.robotpose.pose.pose.orientation.y = quaternion_robot[1]
						self.robotpose.pose.pose.orientation.z = quaternion_robot[2]
						self.robotpose.pose.pose.orientation.w = quaternion_robot[3]
						theta = euler_from_quaternion(quaternion_robot)[2]
						rospy.loginfo("Camera Global Pose:")
						rospy.loginfo("Position: x={}, y={}, z={}".format(
							self.robotpose.pose.pose.position.x,
							self.robotpose.pose.pose.position.y,
							self.robotpose.pose.pose.position.z))
						rospy.loginfo("Orientation: x={}, y={}, z={}, w={}".format(
							self.robotpose.pose.pose.orientation.x,
							self.robotpose.pose.pose.orientation.y,
							self.robotpose.pose.pose.orientation.z,
							self.robotpose.pose.pose.orientation.w))
						self.robotpose.pose.covariance = [100, 250, 0, 0, 0, 0, \
														250, 500, 0, 0, 0, 0, \
														0, 0, 0, 0, 0, 0, \
														0, 0, 0, 0, 0, 0, \
														0, 0, 0, 0, 0, 0, \
														0, 0, 0, 0, 0, 0]
		else:
			self.published = False

	
if __name__ == '__main__':
	rospy.init_node('camera_localise_node')
	detection = FindRobot()
	rospy.Subscriber("/tag_detections",AprilTagDetectionArray,detection.send_camera_pose)
	pub = rospy.Publisher('/aprtag',PoseWithCovarianceStamped,queue_size=1)
	br = tf.TransformBroadcaster()
	R = rospy.Rate(10)
	while not rospy.is_shutdown():
		if detection.published and (abs(detection.position_camera[0])+abs(detection.position_camera[1]))>=1.0:
			rospy.loginfo("Camera Global Pose:")
			rospy.loginfo("Position: x={}, y={}, z={}".format(detection.robotpose.pose.pose.position.x,
															detection.robotpose.pose.pose.position.y,
															detection.robotpose.pose.pose.position.z))
			rospy.loginfo("Orientation: x={}, y={}, z={}, w={}".format(detection.robotpose.pose.pose.orientation.x,
																		detection.robotpose.pose.pose.orientation.y,
																		detection.robotpose.pose.pose.orientation.z,
																		detection.robotpose.pose.pose.orientation.w))
			
			# if :
			# 	self.published = False
			# rospy.loginfo("publishing robot pose")
			# br.sendTransform(detection.position_camera,detection.quaternion_camera,rospy.Time.now(),"usb_cam","world")
			pub.publish(detection.robotpose)
		R.sleep()
	
