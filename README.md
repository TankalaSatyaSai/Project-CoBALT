# THE MINIONS 


## Problem Statement :  The existing warehouse logistics system faces challenges in optimizing the efficiency of package transportation, particularly in scenarios with dynamic obstacles and varying package sizes. There is a need for an advanced robotic solution that can automatically and collaboratively handle package transportation within the warehouse, ensuring efficient obstacle avoidance, path planning, and dynamic coordination based on package characteristics.

## Our Approach : 'The Minions' is a project based on swarm robotics. In this, multiple warehouse-friendly robots are designed to receive packages from users and transport them to specific destinations in a predetermined order. The robots navigate through paths that are randomly chosen, ensuring efficient obstacle avoidance and minimizing the total distance traveled. Depending on the size of the package, the swarm robots dynamically collaborate, forming patterns with two, three, or all four robots working together to transport the package in a coordinated manner to its destination.


## About the Team And CFI :

## Electronic:
- [Anirudhha](#Aniruddha)
- [Justin](#Justin)


## Mechanical:
- [Satya](#Satya)
- [Tejaswini](#Tejaswini)

## Software:
- [Atharva](https://github.com/TankalaSatyaSai/minions/tree/main/Atharva)
- [Devesh](https://github.com/TankalaSatyaSai/minions/tree/main/Devesh)
- [Harshini](https://github.com/TankalaSatyaSai/minions/tree/main/Harshini)
- [Praneeth](https://github.com/TankalaSatyaSai/minions/tree/main/Praneeth)
- [Vidit](https://github.com/TankalaSatyaSai/minions/tree/mainVidit) 

## Why did we choose that solution: We chose Swarm robotics because it excels in adapting to dynamic environments, making it suitable for warehouses with changing layouts, varying package sizes, and dynamic obstacles. The adaptive nature ensures robust performance in real-world scenarios. This leads to optimized path planning, efficient obstacle avoidance, and minimized total distance traveled, contributing to overall efficiency in package transportation.


## Software Module :
### In software module we majorly focus on enabling effective communication and coordination among the swarm robots which needs path planning using global and local planners, computer vision, localisation and mapping. Path planning is vital for delivering packages and are responsible for determining the optimal paths for each robot to navigate through the environment, avoiding obstacles, and reaching their destinations while minimizing travel time. Computer vision is needed as it helps the robots in identifying obstacles, plan optimal paths, and avoid collisions during package delivery. Accurate localization and mapping algorithms is crucial for effective coordination and navigation, especially in scenarios where precise package delivery is required. Key features covered includes A* and RRT* algorithm, MPC, Yolo models, extended kalman filters,2-D LIDAR SLAM algorithms

### Past Works

- Checked how basic robot works in rviz,gazebo
- Codes for Communication between jetson nano and arduino
- Research on various object detection algorithms
- Kinematic model for robot's wheels angular velocities
- Tested the codes with arduino, encoders and motors
- Research on various 2D-LIDAR SLAM algorithms
- Created a map using rpLidar and Hector SLAM algorithm
- Implemented the other lidar slam algorithms as well and figure out which one works best for our project
- Research of sensor fusion and various algorithms needed for it like Kalmann, IMM,Particle Filters etc.
- Codes for ultrasonic ,rplidar, april tag.
- Codes for communication between arduino, ros and ultrasonics without buffer and made launch files.
- Global planners A* and RRT* are done with the codes, yet to be tested.

### Current Works


- To complete the local planner code, test the global planner codes and  build the navigation stack
- image processing that be able to track the objects, humans, apriltags
- Working on the multi system communication codes and the swarm codes
- To complete the whole localisation stack
- Swarm tech in a simulated environment

### Future Works

- Swarm and multiple robots coordination in the warehouse environment
- Human and robot collaboration where we may use interfaces to communicate goals, preferences, or changes in the task to the robot swarm, allowing for dynamic adjustments in real-time.
- Usage of Joy stick and Joy nodes

## Codes and their execution :
### 1.For communication between RPI and controller : In the /Devesh/control_and_comminication folder execute 
```bash
python3 rpi_vel_code.py

```
### this file will establish the serial communication between them. These file will be used to send the velocity inputs to the controller, based on the kinematic modeling of the wheels.In the same folder the file “ new_motor_arduino_ino.ino “ will be programmed into the controller, it contains the pid control.

### 2.Robot Description : The folder “ robot description” contains the meshes and urdf code of the robot, it is equipped with ultrasonic sensors and lidar. We need to include that urdf file in the launch file and execute that file in gazebo.

### 3.Follow curve : Based on the waypoints we have the script “ waypoint_methond.py “ in /Devesh/follow_curve folder which moves robot from one point to another.Run following commands for execution :
```bash
python3 waypoinst.py
rosrun <package_name>  waypoint_methond.py
```

### 4.Global Planner :  For global planner we have two nodes written A* and RRT*. In order to execute those nodes we need “/map” topic.install turtlebot3 package and then execute
```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch # this will create the /map topic using slam
rosrun <package_name>  RRT_star.py # For rrt* algo
rosrun  <package_name> a_star.py # For a* algo
```

### 5.Camera Calibration : In linux terminal type the following commands
```bash
sudo apt-get install ros-noetic-perception
sudo apt-get install ros-noetic-usb-cam
```
### To changes the webcam/camera ,Go to Files—>other location--->Computer—>opt—>ros→noetic—>share—>usb-cam—>launch Using command
```bash
sudo nano usb_cam-test.launch
```
### You can change the camera by changing the number video0/video1/video2 depending upon the availability.To check available cameras
```bash
ls /dev/ | grep video
```
### To do camera calibration run,
``` bash
roslaunch usb_cam usb_cam-test.launch
rosrun camera_calibration cameracalibrator.py –size 8x6 –square 0.024 image:=/usb_cam/image_raw camera:=/usb_cam
```
### Calibrate by moving the chessboard using various orientations.Open hidden files in home
```bash
/atharva/.ros/log/camera_info
```
### Create a head_camera.yaml file and copy the calibrated data in specific format.To check Rostopic list In this check if /usb_cam/camera_info is present or not To run
```bash
Rostopic echo /usb_cam/camera_info
```
### 6.RPLidar:  We are using the RPLidar a1 model. Clone the slamtec GitHub repo to your src folder in catkin_ws using the command
```bash
git clone https://github.com/Slamtec/rplidar_ros.git #( this is the rplidar_ros package)
```

### Then do catkin_make in catkin_ws and connect the rplidar to your laptop ,Add the authority to write to the USB port using: 
```bash
sudo chmod 666 /dev/ttyUSB0
```
### There are two ways to get the point cloud from the lidar
1. using the view_rplidar_a1.launch file, Just run the command: 
```bash
roslaunch rplidar_ros view_rplidar_a1.launch
```

2. using the rplidar_a1.launch file Run the command: 
```bash
roslaunch rplidar_ros rplidar_a1.launch
```
### In another terminal run: rosrun rviz rviz Rviz opens and on the left, you can see some settings there click on add -> laserscan, Then set the topic as /scan and the frame as laser now you can see the point cloud.

### 6. Mapping using Hector SLAM: Clone the GitHub repo below to your src in catkin_ws
```bash
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git 
```

### Then you need to make some changes in the launch files to use Hector Slam without odometry data
- catkin_ws/src/rplidar_hector_slam/hector_slam/hector_mapping/launch/mapping_default.launch
### replace the second last line with
```xml
<node pkg="tf" type="static_transform_publisher" name="base_to_laser_broadcaster" args="0 0 0 0 0 0 base_link laser 100" />
<!--the third line with --> <arg name="base_frame" default="base_link"/>
<!--the fourth line with --> <arg name="odom_frame" default="base_link"/>
```
### In catkin_ws/src/rplidar_hector_slam/hector_slam/hector_slam_launch/launch/tutorial.launch
```xml
<!--replace the third line with --> <param name="/use_sim_time" value="false"/>
```
### Do catkin_make, connect the rplidar and run
``` bash
sudo chmod 666 /dev/ttyUSB0
roslaunch rplidar_ros rplidar_a1.launch
roslaunch hector_slam_launch tutorial.launch
```
### Rviz will open with the map, now move the lidar to get the map. To save the map : 
```bash
sudo apt-get install ros-noetic-map-server
mkdir ~/catkin_ws/maps
cd ~/catkin_ws/maps
rosrun map_server map_saver -f my_map
```







