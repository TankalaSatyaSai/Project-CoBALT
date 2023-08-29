
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from filterpy.kalman import ExtendedKalmanFilter

class SensorFusionNode:
    def __init__(self):
        rospy.init_node('sensor_fusion_node')
        self.lidar_sub = rospy.Subscriber('/lidar_topic', LaserScan, self.lidar_callback)
        self.ultrasonic_sub = rospy.Subscriber('/ultrasonic_topic', Float32, self.ultrasonic_callback)
        self.fused_pub = rospy.Publisher('/fused_distance', Float32, queue_size=10)
    # Initialize the EKF
        self.ekf = ExtendedKalmanFilter(dim_x=2, dim_z=1)
        self.ekf.x = np.array([0.0, 0.0]) # Initial state [distance, velocity]
        self.ekf.F = np.array([[1.0, 1.0], [0.0, 1.0]]) # State transition matrix
        self.ekf.H = np.array([[1.0, 0.0]]) # Measurement function
        self.ekf.P *= 1.0 # Initial covariance matrix
        self.ekf.R = np.array([[0.1]]) # Measurement noise
        self.ekf.Q = np.array([[0.01, 0.01], [0.01, 0.01]]) # Process noise
        #ExtendedKalmanFilter class maintains a state vector (x), a state transition matrix (F), a measurement function (H), a covariance matrix (P), and noise matrices (R for measurement noise and Q for process noise)
        #here x is the controlling value vector, F is the model of the relationship between the current state and the next predicted state
        self.lidar_distance = None
        self.ultrasonic_distance = None

def lidar_callback(self, data):
# Assuming the LIDAR publishes a range value (distance) from the center
    self.lidar_distance = min(data.ranges)
    self.perform_fusion_and_publish()
    def ultrasonic_callback(self, data):
        self.ultrasonic_distance = data.data
        self.perform_fusion_and_publish()

def perform_fusion_and_publish(self):
    if self.lidar_distance is not None and self.ultrasonic_distance is not None:
        z = np.array([[self.ultrasonic_distance]])
        self.ekf.predict()
        self.ekf.update(z)
        fused_distance = self.ekf.x[0]
        self.fused_pub.publish(fused_distance)
        rospy.loginfo("Fused Distance (EKF): %.2f" % fused_distance)

if __name__ == '__main__':
    try:
        node = SensorFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass