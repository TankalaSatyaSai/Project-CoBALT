#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import csv

class DataCollector:
    def __init__(self):
        rospy.init_node('data_collector')
        self.lidar_sub = rospy.Subscriber('/laser/scan', LaserScan, self.lidar_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.lidar_data = []
        self.cmd_vel_data = []

    def lidar_callback(self, data):
        # Process Lidar data and append it to lidar_data list
        self.lidar_data.append(data)

    def cmd_vel_callback(self, data):
        # Process cmd_vel data and append it to cmd_vel_data list
        self.cmd_vel_data.append(data)

    def save_data(self):
        with open('lidar_data.csv', 'w') as lidar_file:
            lidar_writer = csv.writer(lidar_file)
            for data in self.lidar_data:
                # Write Lidar data to CSV file
                # Modify this part based on the structure of LaserScan message
                # For example, writing ranges data:
                lidar_writer.writerow(data.ranges)

        with open('cmd_vel_data.csv', 'w') as cmd_vel_file:
            cmd_vel_writer = csv.writer(cmd_vel_file)
            for data in self.cmd_vel_data:
                # Write cmd_vel data to CSV file
                # Modify this part based on the structure of Twist message
                # For example, writing linear and angular velocities:
                cmd_vel_writer.writerow([data.linear.x, data.angular.z])

    def run(self):
        rospy.spin()
        # Save data when shutting down
        self.save_data()

if __name__ == '__main__':
    data_collector = DataCollector()
    data_collector.run()
