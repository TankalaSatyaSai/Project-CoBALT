# Follow a given Curve given in 2D (y=f(x))

## This includes some steps to follow while launching the simulation.

### Launching the launch file of the mecanum wheel robot.
```bash

  roslaunch nexus_4wd_mecanum_gazebo nexus_4wd_mecanum_gazebo_world.launch

```

### After launching the urdf file , run the node that is written to follow a given curve
```bash
  rosrun nexus_4wd_mecanum_gazebo waypoint_method.py

```

### The waypoint_method.py file initializes the node that subscribes to the '/odom' topic and publishes on the 'cmd_vel' topic.
```python
  
   def main_fun():
    global target_x, target_y,a,b
    rospy.init_node('follow_the_curve')
    rospy.Subscriber('/odom', Odometry, callback=calculate_vel)
    rate = rospy.Rate(10)

   
    rospy.spin()

   pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
   cmd_vel = Twist()

```

### It imports the two arrays, array of x_points and array of y_points from another file waypoints.py

### waypoints.py
```python
   
   #let the curve be y = f(x) , upto x = 100 (we can change accroding to its convinience)

import math


waypoint_x = []  # Initialize as an empty list
waypoint_y = []  # Initialize as an empty list
x = 0.1


def waypoints():
    global x  # Declare x as a global variable
    global y  # Declare y as a global variable
    for i in range(0, 100):
        y = 4**x
        
        waypoint_x.append(x)
        waypoint_y.append(y)
        x = x+0.1

waypoints()

```

### The waypoint_method.py file contains a function that calculates the cmd_vel to be publish
```python

def calculate_vel(msg):
    global target_x, target_y, a, b, prev_err_x, prev_err_y

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd_vel = Twist()
    
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    x_diff = target_x - current_x
    y_diff = target_y - current_y

    # Set linear velocities based on the difference
    cmd_vel.linear.x = 0.2 + 0.5*x_diff + 0.2*(x_diff-prev_err_x) if x_diff > 0 else -0.2 + 0.5*x_diff + 0.2*(x_diff-prev_err_x) if x_diff < 0 else 0.0
    cmd_vel.linear.y = 0.2 + 0.5*y_diff + 0.2*(y_diff-prev_err_y) if y_diff > 0 else -0.2 + 0.5*y_diff + 0.2*(y_diff-prev_err_y) if y_diff < 0 else 0.0

    # Check if the robot is close to the target waypoint
    distance_threshold = 0.05  # Adjust this value as needed
    if abs(x_diff) < distance_threshold and abs(y_diff) < distance_threshold:
        if a < len(waypoint_x) and b < len(waypoint_y):
            target_x = waypoint_x[a]
            target_y = waypoint_y[b]
            a += 1
            b += 1

```

### For demonstration purpose we used the y=4**x eq

### The graph of target_x points and current_x points is 
```
![Error in loading the file](URL)

```


### The graph of target_y points and current_y points is 
```
![Error in loading the file](URL)

```
