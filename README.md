# CoBALT: Collaborative Bots for Automated Logistics and Transport


## Problem Statement:  The project aims to revolutionize warehouse operations by developing a cutting-edge automation system employing mobile robots equipped with mecanum wheels. Leveraging swarm technology for collaborative task execution and autonomous navigation, the system will optimize inventory management, streamline order fulfillment processes, and enhance overall warehouse efficiency.

Traditional warehouse systems face inefficiencies in inventory management, flexibility, order fulfillment, navigation, and scalability. Manual processes and fixed automation systems lead to errors, delays, and high operational costs. These challenges hinder warehouses from meeting the demands of modern commerce.

Proposed Solution:
We aim to develop an innovative warehouse automation system using mobile robots with mecanum wheels and swarm technology. This system will optimize inventory management, order fulfillment, and warehouse operations through autonomous navigation and intelligent robot collaboration.

Key Objectives:
1. Develop a modular and scalable warehouse automation platform.
2. Design autonomous navigation algorithms for dynamic warehouse environments.
3. Implement swarm intelligence principles for robot collaboration.
4. Optimize inventory management and order fulfillment processes.
5. Ensure cost-effectiveness and ROI by leveraging existing technologies.

By achieving these objectives, our solution seeks to overcome the limitations of traditional warehouse systems and revolutionize warehouse operations for increased efficiency and agility.

## About the Team And CFI :

## Mechatronics:
- [Satya Sai](https://github.com/TankalaSatyaSai/minions/tree/main/Satya)
- [Tejaswini](https://github.com/TankalaSatyaSai/minions/tree/main/Tejaswini)

## Software:
- [Satya Sai](https://github.com/TankalaSatyaSai/minions/tree/main/Satya)
- [Atharva](https://github.com/TankalaSatyaSai/minions/tree/main/Atharva)
- [Devesh](https://github.com/TankalaSatyaSai/minions/tree/main/Devesh)
- [Harshinni](https://github.com/TankalaSatyaSai/minions/tree/main/Harshinni)
- [Praneeth](https://github.com/TankalaSatyaSai/minions/tree/main/Praneeth)
- [Vidit](https://github.com/TankalaSatyaSai/minions/tree/mainVidit) 

## Why did we choose that solution? We chose Swarm Robotics because it excels in adapting to dynamic environments, making it suitable for warehouses with changing layouts, varying package sizes, and dynamic obstacles. The adaptive nature ensures robust performance in real-world scenarios. This leads to optimized path planning, efficient obstacle avoidance, and minimized total distance traveled, contributing to overall efficiency in package transportation.


## Software Module :
### In the software module, we focus on enabling effective communication and coordination among the swarm robots, which needs path planning using global and local planners, computer vision, localization, and mapping. Path planning is vital for delivering packages and determines the optimal paths for each robot to navigate the environment, avoiding obstacles and reaching their destinations while minimizing travel time. Computer vision is needed as it helps the robots identify obstacles, plan optimal paths, and avoid collisions during package delivery. Accurate localization and mapping algorithms are crucial for effective coordination and navigation, especially when precise package delivery is required. Key features include A* and RRT* algorithms, MPC, Yolo models, extended Kalman filters, and 2-D LIDAR SLAM algorithms.

### Past Works

- Checked how a basic robot works in rviz, gazebo
- Codes for Communication between Jetson Nano and Arduino
- Research on various object detection algorithms
- Kinematic model for robot's wheels angular velocities
- Tested the codes with Arduino, encoders, and motors
- Research on various 2D-LIDAR SLAM algorithms
- Created a map using rpLidar and Hector SLAM algorithm
- Implemented the other lidar slam algorithms as well and figured out which one works best for our project
- Research of sensor fusion and various algorithms needed for it like Kalman, IMM, Particle Filters, etc.
- Online simulation of objects in Coppeliasim software
- Codes for ultrasonic,rplidar, and april tag.
- Codes for communication between Arduino, ros, and ultrasonics without buffer and made launch files.
- Global planners A* and RRT* are done with the codes and have yet to be tested.

### Current Works

- To complete the local planner code, test the global planner codes, and  build the navigation stack
- image processing that can track the objects, humans, April tags
- Working on the multi-system communication codes and the swarm codes
- To complete the whole localization stack
- Swarm tech in a simulated environment

### Future Works

- Swarm and multiple robots coordination in the warehouse environment
- Human and robot collaboration, where we may use interfaces to communicate goals, preferences, or changes in the task to the robot swarm, allowing for dynamic adjustments in real-time.
- Usage of Joy stick and Joy nodes

### Various swarm algorithms researched

- **Ant Colony Optimization (ACO)**:
        The foraging behavior of ants inspires ACO. Ants deposit pheromones to communicate with each other, and the algorithm uses these artificial pheromones to find optimal paths in combinatorial optimization problems.
  
- **Particle Swarm Optimization** :
        The social behavior of bird flocks and fish schools inspires this. Particles (representing potential solutions robots in our case) adjust their positions based on their own experience and the best solution found by any particle in the swarm.

- **Artificial Bee Colony** :
        Inspiration from Bee Foraging. The algorithm simulates the exploration-exploitation trade-off observed in bees' search for food sources.

- **Bacterial Foraging Optimization (BFO)**:
        BFO models the chemotactic movement of bacteria towards nutrient-rich areas. The algorithm simulates the bacterial foraging process, including chemotaxis, reproduction, and elimination-dispersal events.

- **Firefly Algorithm** :
        Bioluminescent Communication: The Firefly Algorithm is inspired by the flashing behavior of fireflies. Fireflies use bioluminescence to attract mates, and the algorithm models the optimization process based on the attractiveness of solutions.

- **Wolf Pack Algorithm**:
       The cooperative hunting behavior of wolf packs inspires WPA. It involves alpha, beta, and omega wolves representing the best, intermediate, and worst solutions in the optimization space.

- **Fish Schooling Algorithm** :
        FSA is inspired by the collective behavior of fish in schools. It mimics the coordinated movement of fish to optimize solutions through interactions between individuals in the algorithmic school.

- **Bee Colony Algorithm** :
        The swarming behavior of honeybees inspires BCA. It mimics the collective decision-making process of bees when choosing a new nest location during swarming.
               
- **Plant Growth Algorithm (PGA)**:
        PGA simulates the growth and development process observed in plants. The algorithm models the growth of roots, stems, and leaves to optimize solutions in a search space.

- **Spider Monkey Optimization (SMO)**:
         The foraging behavior of spider monkeys inspires SMO. The algorithm mimics their movements in search of optimal food sources and exploration and exploitation.

- **Cuckoo Search Algorithm** :
        The brood parasitism of cuckoo birds inspires CSA. The algorithm includes a mechanism where sure eggs (solutions) are laid by host birds (exploited solutions), simulating the optimization process.

- **Fungal Foraging Optimization Algorithm** :
        The foraging behavior of fungi inspires FOA. The algorithm simulates fungal mycelium's growth and branching patterns to optimize solutions in the search space.

- **Algae-Inspired Algorithm (AIA)**:
        AIA draws inspiration from the photosynthetic and growth processes of algae. The algorithm incorporates mechanisms inspired by algae behavior to optimize solutions in a search space.

- **A * RRT (A-Star Rapidly Exploring Random Tree)**:
        Combination of A and RRT:* A* RRT is a hybrid algorithm combining the A* search algorithm and Rapidly Exploring Random Trees (RRT). It aims to find an optimal path in a space efficiently by using branches of trees ideas
        
- **Flock of Birds Algorithm**:
        Collective Movement of Birds: FBA is inspired by the coordinated movement of bird flocks. The algorithm mimics the collective behavior of birds in a flock to optimize solutions through interactions between individuals.

## Codes and their execution :
### 1.For communication between RPI and controller: In the /Devesh/control_and_comminication folder, execute 
```bash
python3 rpi_vel_code.py

```
### This file will establish the serial communication between them. This file will be used to send the velocity inputs to the controller based on the kinematic modeling of the wheels. In the same folder the file “ new_motor_arduino_ino.ino “ will be programmed into the controller, it contains the pid control.

### 2.Robot Description: The folder “ robot description” contains the robot's meshes and urdf code, and it is equipped with ultrasonic sensors and lidar. We must include that urdf file in the launch file and execute that file in the gazebo.

### 3. Follow curve: Based on the waypoints, we have the script “ waypoint_methond.py “ in the/Devesh/follow_curve folder, which moves the robot from one point to another. Run the following commands for execution :
```bash
python3 waypoinst.py
rosrun <package_name>  waypoint_methond.py
```

### 4. Global Planner: For the global planner, we have two nodes, A* and RRT*. To execute those nodes we need “/map” topic.install turtlebot3 package and then execute
```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch # this will create the /map topic using slam
rosrun <package_name>  RRT_star.py # For rrt* algo
rosrun  <package_name> a_star.py # For a* algo
```

### 5.Camera Calibration: In Linux terminal type the following commands
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







