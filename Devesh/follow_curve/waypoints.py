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
print(waypoint_x)
print(waypoint_y)

        
  



