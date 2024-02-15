# # import math
# # import heapq

# # # Define the Cell class
# # class Cell:
# # 	def __init__(self):
# # 		self.parent_i = 0 # Parent cell's row index
# # 		self.parent_j = 0 # Parent cell's column index
# # 		self.f = float('inf') # Total cost of the cell (g + h)
# # 		self.g = float('inf') # Cost from start to this cell
# # 		self.h = 0 # Heuristic cost from this cell to destination

# # # Define the size of the grid
# # ROW = 9
# # COL = 10

# # # Check if a cell is valid (within the grid)
# # def is_valid(row, col):
# # 	return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

# # # Check if a cell is unblocked
# # def is_unblocked(grid, row, col):
# # 	return grid[row][col] == 1

# # # Check if a cell is the destination
# # def is_destination(row, col, dest):
# # 	return row == dest[0] and col == dest[1]

# # # Calculate the heuristic value of a cell (Euclidean distance to destination)
# # def calculate_h_value(row, col, dest):
# # 	return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5

# # # Trace the path from source to destination
# # def trace_path(cell_details, dest):
# # 	print("The Path is ")
# # 	path = []
# # 	row = dest[0]
# # 	col = dest[1]

# # 	# Trace the path from destination to source using parent cells
# # 	while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
# # 		path.append((row, col))
# # 		temp_row = cell_details[row][col].parent_i
# # 		temp_col = cell_details[row][col].parent_j
# # 		row = temp_row
# # 		col = temp_col

# # 	# Add the source cell to the path
# # 	path.append((row, col))
# # 	# Reverse the path to get the path from source to destination
# # 	path.reverse()

# # 	# Print the path
# # 	for i in path:
# # 		print("->", i, end=" ")
# # 	print()

# # # Implement the A* search algorithm
# # def a_star_search(grid, src, dest):
# # 	# Check if the source and destination are valid
# # 	if not is_valid(src[0], src[1]) or not is_valid(dest[0], dest[1]):
# # 		print("Source or destination is invalid")
# # 		return

# # 	# Check if the source and destination are unblocked
# # 	if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
# # 		print("Source or the destination is blocked")
# # 		return

# # 	# Check if we are already at the destination
# # 	if is_destination(src[0], src[1], dest):
# # 		print("We are already at the destination")
# # 		return

# # 	# Initialize the closed list (visited cells)
# # 	closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
# # 	# Initialize the details of each cell
# # 	cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

# # 	# Initialize the start cell details
# # 	i = src[0]
# # 	j = src[1]
# # 	cell_details[i][j].f = 0
# # 	cell_details[i][j].g = 0
# # 	cell_details[i][j].h = 0
# # 	cell_details[i][j].parent_i = i
# # 	cell_details[i][j].parent_j = j

# # 	# Initialize the open list (cells to be visited) with the start cell
# # 	open_list = []
# # 	heapq.heappush(open_list, (0.0, i, j))

# # 	# Initialize the flag for whether destination is found
# # 	found_dest = False

# # 	# Main loop of A* search algorithm
# # 	while len(open_list) > 0:
# # 		# Pop the cell with the smallest f value from the open list
# # 		p = heapq.heappop(open_list)

# # 		# Mark the cell as visited
# # 		i = p[1]
# # 		j = p[2]
# # 		closed_list[i][j] = True

# # 		# For each direction, check the successors
# # 		directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
# # 		for dir in directions:
# # 			new_i = i + dir[0]
# # 			new_j = j + dir[1]

# # 			# If the successor is valid, unblocked, and not visited
# # 			if is_valid(new_i, new_j) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
# # 				# If the successor is the destination
# # 				if is_destination(new_i, new_j, dest):
# # 					# Set the parent of the destination cell
# # 					cell_details[new_i][new_j].parent_i = i
# # 					cell_details[new_i][new_j].parent_j = j
# # 					print("The destination cell is found")
# # 					# Trace and print the path from source to destination
# # 					trace_path(cell_details, dest)
# # 					found_dest = True
# # 					return
# # 				else:
# # 					# Calculate the new f, g, and h values
# # 					g_new = cell_details[i][j].g + 1.0
# # 					h_new = calculate_h_value(new_i, new_j, dest)
# # 					f_new = g_new + h_new

# # 					# If the cell is not in the open list or the new f value is smaller
# # 					if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
# # 						# Add the cell to the open list
# # 						heapq.heappush(open_list, (f_new, new_i, new_j))
# # 						# Update the cell details
# # 						cell_details[new_i][new_j].f = f_new
# # 						cell_details[new_i][new_j].g = g_new
# # 						cell_details[new_i][new_j].h = h_new
# # 						cell_details[new_i][new_j].parent_i = i
# # 						cell_details[new_i][new_j].parent_j = j

# # 	# If the destination is not found after visiting all cells
# # 	if not found_dest:
# # 		print("Failed to find the destination cell")

# # def main():
# # 	# Define the grid (1 for unblocked, 0 for blocked)
# # 	grid = [
# # 		[1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
# # 		[1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
# # 		[1, 1, 1, 0, 1, 1, 0, 1, 0, 1],
# # 		[0, 0, 1, 0, 1, 0, 0, 0, 0, 1],
# # 		[1, 1, 1, 0, 1, 1, 1, 0, 1, 0],
# # 		[1, 0, 1, 1, 1, 1, 0, 1, 0, 0],
# # 		[1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
# # 		[1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
# # 		[1, 1, 1, 0, 0, 0, 1, 0, 0, 1]
# # 	]

# # 	# Define the source and destination
# # 	src = [8, 0]
# # 	dest = [0, 0]

# # 	# Run the A* search algorithm
# # 	a_star_search(grid, src, dest)

# # if __name__ == "__main__":
# # 	main()

# # from astar import xy_coor
# # import csv
# # print(xy_coor)
# # for i in range(len(xy_coor)):
# #     print(xy_coor[0],' ', xy_coor[1])


# #!/usr/bin/env python3
# #importing the necessary Libraries required 

# import os # python library for system file path

# import numpy as np # the numerical python library

# import rospy #python library for ros
# from geometry_msgs.msg import Twist #importing the messgae for publishing 
# from geometry_msgs.msg import PoseWithCovarianceStamped #importig message for publish
# from tf.transformations import euler_from_quaternion # feature necesssary for converting the quaternion to euler angles

# import matplotlib #for plotting 
# import matplotlib.transforms as transforms #for error ellipse transforms
# import matplotlib.pyplot as plt #for plotting 
# from matplotlib.patches import Ellipse #for plotting Ellipse

# #Setup paths
# cwd_path = os.path.dirname(os.path.abspath(__file__)) #gettting the parent directory path

# #Setup plot
# plt.ion()# seeting up interactive mode

# #Store robot pose
# robot_pose  = [5.0, 3.0, 0.0] #x, y position and yaw angle 

# #Defining two axes and figures for plotting 
# fig, ax1 = plt.subplots()

# #Plot map as a backgorund image 
# file_path = os.path.join(cwd_path, '/home/devesh/catkin_ws/src/contorl_and_communication/map/map_3rd_floor.jpeg') #finding full file path 
# img = plt.imread(file_path) #reading the background image 
# ax1.imshow(img)

# #Figure setup
# ax1.set(xlabel='X(m)', ylabel='Y(m)') #setting X and Y labels 
# matplotlib.rcParams['legend.fontsize'] = 15 #updating the font
# x_offset = 93 # x offset to adjust with the map, home coordinates
# y_offset = 200 # y offset to adjust with the map, home coordinates
# scale_x = 60.30 # scale for x values to adjust with the map
# scale_y = 60.30 # scale for y values to adjust with the map
# heading_line_length = 25.0 #length of the black geading line

# #Plot robot
# robot_x_plot = (robot_pose[0]/scale_x) + x_offset
# robot_y_plot = (robot_pose[1]/scale_y) + y_offset
# robot_point_plot_handle = ax1.scatter(robot_x_plot, robot_y_plot, alpha=1.0, s=50, color='green') # plotting the robot as a point ( position from ekf )

# #Plot trail
# prev_robot_x_plot = robot_x_plot
# prev_robot_y_plot = robot_y_plot
# ax1.plot([robot_x_plot, prev_robot_x_plot], [robot_y_plot, prev_robot_y_plot],'k--') 

# #Plot heading
# heading_x_plot = [robot_x_plot, robot_x_plot + heading_line_length*np.cos(robot_pose[2])]
# heading_y_plot = [robot_y_plot, robot_y_plot + heading_line_length*np.sin(robot_pose[2])]
# heading_line_plot_handle, = ax1.plot(heading_x_plot, heading_y_plot,'g-') 

# #Plot error ellipse
# error_ellipse_plot = Ellipse(xy = (robot_x_plot, robot_y_plot), width = 1.0, height = 1.0, angle = 0.0, edgecolor='y', fc='None', lw=2)
# ax1.add_patch(error_ellipse_plot)

# #Set legend
# ax1.legend()

# def ekf_callback(data):
#     """A callback funcction which plots the updated ekf position and error ellipse"""
#     global error_ellipse_plot
#     global prev_robot_x_plot 
#     global prev_robot_y_plot 

#     #Update the robot pose from message
#     robot_pose[0] = data.pose.pose.position.x
#     robot_pose[1] = data.pose.pose.position.y

#     #Converting the quaternion to euler angle 
#     measurement_quat = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
#     _,_,robot_pose[2] = euler_from_quaternion(measurement_quat)


#     #Update robot position plot
#     robot_x_plot = (robot_pose[1]/scale_x) + x_offset
#     robot_y_plot = (robot_pose[0]/scale_y) + y_offset
#     robot_point_plot_handle.set_offsets([robot_x_plot, robot_y_plot])  
    
#     #Update robot trail
#     ax1.plot([robot_x_plot, prev_robot_x_plot], [robot_y_plot, prev_robot_y_plot],'k--') 
#     prev_robot_x_plot = robot_x_plot
#     prev_robot_y_plot = robot_y_plot

#     #Update heading plot 
#     heading_x_plot = [robot_x_plot, robot_x_plot + heading_line_length*np.sin(robot_pose[2])]
#     heading_y_plot = [robot_y_plot, robot_y_plot + heading_line_length*np.cos(robot_pose[2])]
#     heading_line_plot_handle.set_xdata(heading_x_plot)
#     heading_line_plot_handle.set_ydata(heading_y_plot)

#     #Update the error covariance    
#     #Getting the covariance matrix and performing eigen decomposition on it 
#     covariance_from_ekf = np.zeros((3,3))
#     covariance_from_ekf[0,0] = data.pose.covariance[0]
#     covariance_from_ekf[1,1] = data.pose.covariance[7]
#     covariance_from_ekf[2,2] = data.pose.covariance[35]
#     covariance_from_ekf[1,2] = data.pose.covariance[11]
#     covariance_from_ekf[2,1] = data.pose.covariance[31]
#     covariance_from_ekf[0,1] = data.pose.covariance[1]
#     covariance_from_ekf[1,0] = data.pose.covariance[6]
#     covariance_from_ekf[2,0] = data.pose.covariance[30]
#     covariance_from_ekf[0,2] = data.pose.covariance[5]
#     covariance_np_array = np.array(covariance_from_ekf)

#     #Doing eigen decomposition and obtraining the eigen values
#     eigenvals, eigenvects = np.linalg.eig(covariance_np_array[0:2,0:2])
#     error_angle = np.arctan2(eigenvects[1,0], eigenvects[0,0])
#     error_x = np.sqrt(eigenvals[0])
#     error_y = np.sqrt(eigenvals[1])
#     error_ellipse_plot.remove()

#     #plotting the extracted Ellipse
#     error_ellipse_plot = Ellipse(xy = (robot_x_plot, robot_y_plot), width = error_x/scale_x, height = error_y/scale_y, angle = np.rad2deg(robot_pose[2] + error_angle), edgecolor='y', fc='None', lw=2)
#     ax1.add_patch(error_ellipse_plot)
     
# if __name__ == '__main__':

#     #intialising a node for the vizualisation part 
#     rospy.init_node('robot_visualisation')

#     #subscribing the required topic and updating its callback function 
#     rospy.Subscriber("/pose_ekf",PoseWithCovarianceStamped,ekf_callback)

#     #rate
#     rate = rospy.Rate(4)#4Hz
#     plt.show(block=True)
#     rate.sleep()

# import csv
# import numpy as np
# from matplotlib import pyplot as plt
# plt.rcParams["figure.figsize"] = [7.00, 3.50]
# plt.rcParams["figure.autolayout"] = True
# im = plt.imread("/home/devesh/catkin_ws/src/contorl_and_communication/map/map_3rd_floor.jpeg")
# fig, ax = plt.subplots()
# im = ax.imshow(im, extent=[0, 910, 0, 499])
# csv_file = '/home/devesh/catkin_ws/src/nexus_4wd_mecanum_simulator/nexus_4wd_mecanum_gazebo/scripts/global_planner/file.csv'  
# with open(csv_file, 'r') as file:
#     reader = csv.reader(file)
#     # for row in reader:
#     #         #if len(row) == 2:
#     #             rows = str(row[0])
#     #             x,y=rows.split()
#     #             x=int(x)
#     #             y=int(y)
#     #             # # x, y = row[0],row[1]
#     #             ax.plot(x, y, ls='dotted', linewidth=2, color='red')

# x = np.array(range(300))
# ax.plot(x, x, ls='dotted', linewidth=2, color='red')


# plt.show()

import csv
import matplotlib.pyplot as plt

# Load the image
image_path = "/home/devesh/catkin_ws/src/contorl_and_communication/map/map_3rd_floor.jpeg"
image = plt.imread(image_path)

# Create a plot
plt.figure(figsize=(10, 5))
plt.imshow(image)

txtfile = "/home/devesh/catkin_ws/src/nexus_4wd_mecanum_simulator/nexus_4wd_mecanum_gazebo/scripts/global_planner/file.csv"
with open(txtfile, 'r') as file:
    for line in file:
        y, x = map(float, line.strip().split(','))  # Split the string into x and y values and convert to float
        plt.scatter(x, y, s=5, color='red')  # Plot the point


plt.show()