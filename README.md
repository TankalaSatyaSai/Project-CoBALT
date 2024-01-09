# minions
The Minions project is an exciting and innovative endeavor in the field of swarm robotics. Our goal is to develop a swarm of small robots capable of performing package delivery tasks in a coordinated and intelligent manner.

## Problem Statement

The existing warehouse logistics system faces challenges in optimizing the efficiency of package transportation, particularly in scenarios with dynamic obstacles and varying package sizes. There is a need for an advanced robotic solution that can automatically and collaboratively handle package transportation within the warehouse, ensuring efficient obstacle avoidance, path planning, and dynamic coordination based on package characteristics.

## Solution that we're building

'The Minions' is a project based on swarm robotics. In this, multiple warehouse-friendly robots are designed to receive packages from users and transport them to specific destinations in a predetermined order. The robots navigate through paths that are randomly chosen, ensuring efficient obstacle avoidance and minimizing the total distance traveled. Depending on the size of the package, the swarm robots dynamically collaborate, forming patterns with two, three, or all four robots working together to transport the package in a coordinated manner to its destination.


## Our Team

Our team has 3 modules

### Electrical Module
- Anirudh
- Justin

### Mechanical Module
- Satya Sai
- Tejaswini

### Software Module
- Atharva
- Devesh
- Harshinni
- Praneeth
- Vidit

## Why did we choose this solution

We chose Swarm robotics because it excels in adapting to dynamic environments, making it suitable for warehouses with changing layouts, varying package sizes, and dynamic obstacles. The adaptive nature ensures robust performance in real-world scenarios. This leads to optimized path planning, efficient obstacle avoidance, and minimized total distance traveled, contributing to overall efficiency in package transportation.


## Software Module

### What does software module work look like

In software module we majorly focus on enabling effective communication and coordination among the swarm robots which needs path planning using global and local planners, computer vision, localisation and mapping. Path planning is vital for delivering packages and are responsible for determining the optimal paths for each robot to navigate through the environment, avoiding obstacles, and reaching their destinations while minimizing travel time. Computer vision is needed as it helps the robots in identifying obstacles, plan optimal paths, and avoid collisions during package delivery. Accurate localization and mapping algorithms is crucial for effective coordination and navigation, especially in scenarios where precise package delivery is required. Key features covered includes A* and RRT* algorithm, MPC, Yolo models, extended kalman filters,2-D LIDAR SLAM algorithms

### Past works

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

### Current works


- To complete the local planner code, test the global planner codes and  build the navigation stack
- image processing that be able to track the objects, humans, apriltags
- Working on the multi system communication codes and the swarm codes
- To complete the whole localisation stack
- Swarm tech in a simulated environment

