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

def convert_cmd_vel(msg):

    global wheel_radius, robot_width, front_left_w, front_right_w, rear_left_w, rear_right_w, max_speed, min_speed, separation_bet_wheels 
    a = wheel_radius
    d = robot_width
    l = separation_bet_wheels/2
    

    x_vel = msg.linear.x 
    y_vel = msg.linear.y
    w_vel = msg.angular.z

    cmd_vel_matrix = np.array([[x_vel],
                               [y_vel],
                               [w_vel]])
    
    conversion_matrix = np.array([[1, 1, 1, 1],
                                  [1, -1, 1, -1]
                                  [1/(l-d), 1/(l-d), 1/(d-l), 1/(d-l)]])
    
    inverse_conversion_matrix = np.linalg.inv(conversion_matrix)

    wheel_ang_vel = (4/a) * inverse_conversion_matrix * cmd_vel_matrix

    front_left_w = wheel_ang_vel[0][0]
    front_right_w = wheel_ang_vel[1][0]
    rear_left_w = wheel_ang_vel[2][0]
    rear_right_w = wheel_ang_vel[3][0]

    return

    

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






        
    
