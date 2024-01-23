#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
import serial
import time
import numpy as np
# from hw_interface.msg import ticks

#Open the serial connection to the Arduino - which causes the Arduino to reset
ser = serial.Serial("/dev/ttyUSB0", 9600)
#ser = serial.Serial("/dev/ttyUSB1", 9600)

front_left_w, front_right_w, rear_left_w, rear_right_w = 0.0,0.0,0.0,0.0

startMarker = 60 #Unicode code for <
endMarker = 62 #Unicode code for >

#To store information from ROS callback
left_speed = 0
right_speed = 0
max_speed = 10000
min_speed = 0.1
separation_bet_wheels = 50

#Robot physical parms
wheel_radius = 37.0 #mm
robot_width = 475.0 #mm

#============================
def recvFromArduino():
# Receiving a message from the Arduino involves
# waiting until the startMarker is detected
# saving all subsequent bytes until the end marker is detected
  global startMarker, endMarker, ser

  recieved_data = ""
  recieved_char = "!" # any value that is not an endmarker (>) or startMarker (<)
  
  # wait for the start character
  while  ord(recieved_char) != startMarker: 
    recieved_char = ser.read()
  
  # save data until the end marker is found
  while ord(recieved_char) != endMarker:
    if ord(recieved_char) != startMarker:
      recieved_data = recieved_data + recieved_char.decode('utf-8') 
    recieved_char = ser.read()
  
  return(recieved_data)

#============================
# Wait for a message from the Arduino to give it time to reset
def waitForArduino():

  # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
  # it also ensures that any bytes left over from a previous message are discarded
  
  global startMarker, endMarker, ser
  print("Waiting for Arduino to be ready")
  
  msg = ""
  while msg.find("Arduino is ready") == -1:
    #Wait until something comes into the serial recieve buffer
    while (ser.in_waiting == 0):
      if rospy.is_shutdown(): 
        return False
    
    msg = recvFromArduino()
 
  return True

#===========================
# Function for gracefull shutdown
def turn_off():
  #Function for safe shutdown 
  
  print('hw_interface node turning off')
  ser.write("<0,0>".encode('utf-8'))
  time.sleep(3)
  ser.close()
  return

#===========================
# ROS callback
def convert_vel_cmd(msg):
  #Function to convert linear and angular velocity request to 
  #left and right wheel velocities

  global wheel_radius, robot_width, front_left_w, front_right_w, rear_left_w, rear_right_w, max_speed, min_speed, separation_bet_wheels 
  a = wheel_radius
  d = robot_width
  l = separation_bet_wheels
  

  x_vel = msg.linear.x 
  y_vel = msg.linear.y
  w_vel = msg.angular.z

  # cmd_vel_matrix = np.array([[x_vel],
  #                             [y_vel],
  #                             [w_vel]])
  
  # conversion_matrix = np.array([[1, 1, 1, 1],
  #                               [-1, 1, 1, -1],
  #                               [-1/(l+d), 1/(l+d), -1/(d+l), 1/(d+l)]])
  
  # inverse_conversion_matrix = np.linalg.pinv(conversion_matrix)

  # wheel_ang_vel = (4/a) * np.dot(inverse_conversion_matrix , cmd_vel_matrix)

  # front_left_w = wheel_ang_vel[0][0]
  # front_right_w = wheel_ang_vel[1][0]
  # rear_left_w = wheel_ang_vel[2][0]
  # rear_right_w = wheel_ang_vel[3][0]
  # if abs(w_vel) == 0.0:
  front_left_w = (1/a)*(x_vel-y_vel-(d+l)*w_vel)
  front_right_w = (1/a)*(x_vel+y_vel+(d+l)*w_vel)
  rear_left_w = (1/a)*(x_vel+y_vel-(d+l)*w_vel)
  rear_right_w = (1/a)*(x_vel-y_vel+(d+l)*w_vel)
  # elif w_vel > 0.0:
  #   front_left_w = -abs((1/a)*(x_vel-y_vel-(d+l)*w_vel))
  #   front_right_w = abs((1/a)*(x_vel+y_vel+(d+l)*w_vel))
  #   rear_left_w = abs((1/a)*(x_vel+y_vel-(d+l)*w_vel))
  #   rear_right_w = -abs((1/a)*(x_vel-y_vel+(d+l)*w_vel))
  # elif w_vel < 0.0:
  #   front_left_w = -abs((1/a)*(x_vel-y_vel-(d+l)*w_vel))
  #   front_right_w = -abs((1/a)*(x_vel+y_vel+(d+l)*w_vel))
  #   rear_left_w = -abs((1/a)*(x_vel+y_vel-(d+l)*w_vel))
  #   rear_right_w = abs((1/a)*(x_vel-y_vel+(d+l)*w_vel))

  

  return

#============================
def main():
  global wheel_radius, robot_width, ser, left_speed, right_speed, front_left_w, front_right_w, rear_left_w, rear_right_w

  #Initialising             
  rospy.init_node('hw_interface_node_front_wheels')
  rospy.on_shutdown(turn_off)
  if not waitForArduino(): return 
  
  print('hw_interface node running_front_wheels')
  rospy.Subscriber("/cmd_vel", Twist, convert_vel_cmd)
#   pub_encoder_ticks = rospy.Publisher("/Encoder_Ticks", ticks, queue_size = 1)
  
  wheel_radius = rospy.get_param('wheel_radius', 37)
  robot_width = rospy.get_param('robot_width', 475)

  rate = rospy.Rate(4) 
  while not rospy.is_shutdown():
    
    #Sent commanded velocity to Arduino
    print(front_left_w , front_left_w, rear_left_w, rear_right_w )
    output_string = "<" + str(int(front_left_w*max_speed)) + "," + str(int(front_right_w*max_speed)) + ">"
    ser.reset_output_buffer()
    ser.write(output_string.encode('utf-8'))
   
    #Update ticks info from arduino
    ser.reset_input_buffer()
    #Wait until something comes into the serial recieve buffer
    while (ser.in_waiting == 0): 
      if rospy.is_shutdown():
        return

     

    rate.sleep()

  return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
    