import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
import serial
import time
import math

ser = serial.Serial("/dev/ttyUSB0", 9600)

startMarker = 60 #Unicode code for <
endMarker = 62 #Unicode code for >

front_left_speed = 0
front_right_speed = 0
rear_left_speed = 0
rear_right_speed = 0
max_speed = 4
min_speed = 0.5
wheel_radius = 37.0 
robot_width = 475.0

def received_info():
    global startMarker, endMarker, ser

    received_data = ""
    received_char = "!"

    while ord(received_data) != startMarker:
        received_char = ser.read()
    
    while ord(received_char) != endMarker:
        if ord(received_char) != startMarker:
            received_data = received_data + received_char.decode('utf-8')
        received_char = ser.read()

    return received_data

def wait_for_arduino():
    global startMarker,endMarker,ser
    print("waiting for arduino's message")
    msg = ""
    
    while msg.find("Arduino is ready") == -1:
        
        while ser.in_waiting == 0:
            pass

    msg = received_info()

    print(msg)
    return 

def shut_down():
  
  
  print('hw_interface node turning off')
  ser.write("<0,0>".encode('utf-8'))
  time.sleep(3)
  ser.close()
  return

# def convert_cmd_vel(msg):

#     global wheel_radius, robot_width, left_speed, right_speed, max_speed, min_speed 
    
#     cmd_linear_vel = msg.linear.x*1000.0 # in mm/sec
#     cmd_angular_vel = msg.angular.z # in rad/sec
#     f_r_speed = ((2.0*cmd_linear_vel) + (cmd_angular_vel*robot_width))/(2*wheel_radius) #in rad/sec
#     f_l_speed = ((2.0*cmd_linear_vel) - (cmd_angular_vel*robot_width))/(2*wheel_radius) #in rad/sec
#     r_r_speed = ((2.0*cmd_linear_vel) + (cmd_angular_vel*robot_width))/(2*wheel_radius) #in rad/sec
#     r_l_speed = ((2.0*cmd_linear_vel) - (cmd_angular_vel*robot_width))/(2*wheel_radius) #in rad/sec

     
#     f_r_speed = f_r_speed/(2*3.14) #in revolutions per second
#     f_l_speed = f_l_speed/(2*3.14)
#     r_r_speed = r_r_speed/(2*3.14) #in revolutions per second
#     r_l_speed = r_l_speed/(2*3.14)

#     vel_rl_max = max(f_r_speed,f_l_speed,r_l_speed,r_r_speed)
#     vel_rl_min = min(f_r_speed,f_l_speed,r_l_speed,r_r_speed)

#     if cmd_linear_vel != 0 and cmd_angular_vel != 0:
#         if vel_rl_max > max_speed:
#             f_r_speed = f_r_speed - (vel_rl_max-max_speed)
#             f_l_speed = f_l_speed - (vel_rl_max-max_speed)
#             r_r_speed = r_r_speed - (vel_rl_max-max_speed)
#             r_l_speed = r_l_speed - (vel_rl_max-max_speed)
#         elif vel_rl_max < -max_speed:
#             f_r_speed = f_r_speed - (vel_rl_max+max_speed)
#             f_l_speed = f_l_speed - (vel_rl_max+max_speed)
#             r_r_speed = r_r_speed - (vel_rl_max+max_speed)
#             r_l_speed = r_l_speed - (vel_rl_max+max_speed)

#         if abs(r_l_speed) < min_speed or abs(r_r_speed) < min_speed or abs(f_r_speed) < min_speed or abs(f_l_speed) < min_speed:
#             if abs(f_r_speed - f_l_speed) < 2*min_speed or abs(r_r_speed - r_l_speed) < 2*min_speed:
#                 r_l_speed = math.copysign(min_speed,r_l_speed)
#                 r_r_speed = math.copysign(min_speed,r_r_speed)
#                 f_l_speed = math.copysign(min_speed,f_l_speed)
#                 f_r_speed = math.copysign(min_speed,f_r_speed)
#             else:
#                 if abs(r_l_speed) < min_speed:
#                     speed_sign = math.copysign(min_speed, r_l_speed)
#                     r_r_speed = r_r_speed + (speed_sign - r_l_speed)
#                     r_l_speed = speed_sign
#                 if abs(r_r_speed) < min_speed:
#                     speed_sign = math.copysign(min_speed, r_r_speed)
#                     r_l_speed = r_l_speed + (speed_sign - r_r_speed)
#                     r_r_speed = speed_sign
#                 if abs(f_l_speed) < min_speed:
#                     speed_sign = math.copysign(min_speed, f_l_speed)
#                     f_r_speed = f_r_speed + (speed_sign - f_l_speed)
#                     f_l_speed = speed_sign
#                 if abs(f_r_speed) < min_speed:
#                     speed_sign = math.copysign(min_speed, f_r_speed)
#                     f_l_speed = f_l_speed + (speed_sign - f_r_speed)
#                     f_r_speed = speed_sign

#         front_right_speed = f_r_speed
#         front_left_speed = f_l_speed
#         rear_left_speed = r_l_speed
#         rear_right_speed = r_r_speed
#         return
    

def main_fun():
    global wheel_radius, robot_width, ser, front_left_speed, front_right_speed, rear_left_speed, rear_right_speed

    rospy.init_node("hw_interface")
    rospy.on_shutdown(shut_down())
    # wait_for_arduino()

    print("hw_interface node is running")
    # rospy.Subscriber("/cmd_vel", Twist, convert_cmd_vel)
    pub_front_left_tick = rospy.Publisher("/front_left_ticks", Int64, queue_size = 1)
    pub_front_right_tick = rospy.Publisher("/front_right_tick", Int64, queue_size=1)
    pub_rear_left_tick = rospy.Publisher("/rear_left_ticks", Int64, queue_size = 1)
    pub_rear_right_tick = rospy.Publisher("/rear_right_tick", Int64, queue_size=1)

    wheel_radius = rospy.get_param('wheel_radius', 37)
    robot_width = rospy.get_param('robot_width', 475)

    rate = rospy.Rate(4)

    while not rospy.is_shutdown:

        output_string = "<" + str(int(front_left_speed*100)) + "," + str(int(front_right_speed*100)) + "," + str(int(rear_left_speed*100)) + "," + str(int(rear_right_speed*100)) + ">"
        print(output_string)
        ser.reset_input_buffer()
        ser.write(output_string.encode('utf-8'))

        ser.reset_input_buffer()
        while (ser.in_waiting == 0):
            pass

        received_data = received_info()
        print("a::", received_data)
        ticks = [int(x) for x in received_data.split(',')]

        print("ticks",ticks)

        pub_front_left_tick.publish(ticks[0])
        pub_front_right_tick.publish(ticks[1])
        pub_rear_left_tick.publish(ticks[2])
        pub_rear_right_tick.publish(ticks[3])

        rate.sleep()
        return
    
if __name__=='__main__':
    try:
        main_fun()
    except rospy.ROSInterruptException:
        pass






        
    
