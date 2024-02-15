#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq
import matplotlib.pyplot as plt
import csv
# xy_coor = []


# Define the Cell class
class Cell:
    def __init__(self):
        self.parent_i = 0 # Parent cell's row index
        self.parent_j = 0 # Parent cell's column index
        self.f = float('inf') # Total cost of the cell (g + h)
        self.g = float('inf') # Cost from start to this cell
        self.h = 0 # Heuristic cost from this cell to destination

# Define the size of the grid
ROW = 499
COL = 910
resolution = 0.072

class ASTAR():
    def __init__(self):
        rospy.init_node('a_star_planner')
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        rospy.spin()

    # Check if a cell is valid (within the grid)
    def is_valid(self, row, col):
        return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

    # Check if a cell is unblocked
    def is_unblocked(self, grid, row, col):
        return grid[row][col] == 0

    # Check if a cell is the destination
    def is_destination(self, row, col, dest):
        return row == dest[0] and col == dest[1]

    # Calculate the heuristic value of a cell (Euclidean distance to destination)
    def calculate_h_value(self, row, col, dest):
        return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5

    # Trace the path from source to destination
    def trace_path(self, cell_details, dest):
        global xy_coor
        print("The Path is ")
        path = []
        row = dest[0]
        col = dest[1]

        # Trace the path from destination to source using parent cells
        while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
            path.append((row, col))
            temp_row = cell_details[row][col].parent_i
            temp_col = cell_details[row][col].parent_j
            row = temp_row
            col = temp_col

        # Add the source cell to the path
        path.append((row, col))
        # Reverse the path to get the path from source to destination
        path.reverse()

        # Print the path
        for i in path:
            print("->", i, end=" ")
          

        print()

        # with open('file.csv', 'w', newline='') as csvfile:
        #     csv_writer = csv.writer(csvfile)
        #     csv_writer.writerow(['Row', 'Column'])  # Write header
        #     for coord in path:
        #         csv_writer.writerow(coord)
        with open('file.txt', 'w') as txtfile:
            for coord in path:
                txtfile.write(f"{coord[0]}, {coord[1]}\n")
        
        # xy_coor = path
        

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"  # Set the frame ID according to your map frame

        # Populate the Path message with waypoints
        path = []
        row = dest[0]
        col = dest[1]

        while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
            pose = PoseStamped()
            pose.pose.position.x = col * resolution  # Adjust the position according to your grid and map resolution
            pose.pose.position.y = row * resolution
            path.append(pose)
            temp_row = cell_details[row][col].parent_i
            temp_col = cell_details[row][col].parent_j
            row = temp_row
            col = temp_col

        # Add the source cell to the path
        pose = PoseStamped()
        pose.pose.position.x = col * resolution
        pose.pose.position.y = row * resolution
        path.append(pose)

        # Reverse the path to get the path from source to destination
        path.reverse()

        # Add the waypoints to the Path message
        path_msg.poses = path

        # Publish the Path message
        self.path_pub.publish(path_msg)

    # Implement the A* search algorithm
    def a_star_search(self, grid, src, dest):
        # Check if the source and destination are valid
        if not self.is_valid(src[0], src[1]) or not self.is_valid(dest[0], dest[1]):
            print("Source or destination is invalid")
            return

        # Check if the source and destination are unblocked
        if not self.is_unblocked(grid, src[0], src[1]) or not self.is_unblocked(grid, dest[0], dest[1]):
            print("Source or the destination is blocked")
            return

        # Check if we are already at the destination
        if self.is_destination(src[0], src[1], dest):
            print("We are already at the destination")
            return

        # Initialize the closed list (visited cells)
        closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
        # Initialize the details of each cell
        cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

        # Initialize the start cell details
        i = src[0]
        j = src[1]
        cell_details[i][j].f = 0
        cell_details[i][j].g = 0
        cell_details[i][j].h = 0
        cell_details[i][j].parent_i = i
        cell_details[i][j].parent_j = j

        # Initialize the open list (cells to be visited) with the start cell
        open_list = []
        heapq.heappush(open_list, (0.0, i, j))

        # Initialize the flag for whether destination is found
        found_dest = False

        # Main loop of A* search algorithm
        while len(open_list) > 0:
            # Pop the cell with the smallest f value from the open list
            p = heapq.heappop(open_list)

            # Mark the cell as visited
            i = p[1]
            j = p[2]
            closed_list[i][j] = True

            # For each direction, check the successors
            directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
            for dir in directions:
                new_i = i + dir[0]
                new_j = j + dir[1]

                # If the successor is valid, unblocked, and not visited
                if self.is_valid(new_i, new_j) and self.is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                    # If the successor is the destination
                    if self.is_destination(new_i, new_j, dest):
                        # Set the parent of the destination cell
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j
                        print("The destination cell is found")
                        # Trace and print the path from source to destination
                        self.trace_path(cell_details, dest)
                        found_dest = True
                        return
                    else:
                        # Calculate the new f, g, and h values
                        g_new = cell_details[i][j].g + 1.0
                        h_new = self.calculate_h_value(new_i, new_j, dest)
                        f_new = g_new + h_new

                        # If the cell is not in the open list or the new f value is smaller
                        if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                            # Add the cell to the open list
                            heapq.heappush(open_list, (f_new, new_i, new_j))
                            # Update the cell details
                            cell_details[new_i][new_j].f = f_new
                            cell_details[new_i][new_j].g = g_new
                            cell_details[new_i][new_j].h = h_new
                            cell_details[new_i][new_j].parent_i = i
                            cell_details[new_i][new_j].parent_j = j

        # If the destination is not found after visiting all cells
        if not found_dest:
            print("Failed to find the destination cell")

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        data = msg.data

        # Convert OccupancyGrid data to a numpy array
        grid_data = np.array(data).reshape((height, width))

        # Map values to a binary grid (occupied=1, unknown=0, free=0)
        grid = np.where(grid_data == 100, 1, 0)
        grid = np.flipud(grid)


        # Save the grid to a text file
        file_name = 'grid.txt'
        np.savetxt(file_name, grid, fmt='%d')

        # Set source and destination within the grid bounds
        # src = [45, 50]  # Top-left corner  ??????????????????? NOTE THAT X and Y are altered here,  the first term is row and second is column
        src = [465,60]
        dest = [220, 750]  # Bottom-right corner   

        # In rviz len = 60.5, width = 30.9  and in 910 and 499 in plotting so during plotting factor of 910/60.5 to x and 499/30.9 to y should be multiplied

        # Run the A* search algorithm
        self.a_star_search(grid, src, dest)

if __name__ == "__main__":
    ASTAR()
