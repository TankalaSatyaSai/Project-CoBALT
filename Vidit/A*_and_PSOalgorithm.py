import heapq
import random
import numpy as np

class Node:
    def __init__(self, row, col, cost, heuristic):
        # Initialize a node with its row, column, cost, and heuristic value
        self.row = row
        self.col = col
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = cost + heuristic
        self.parent = None

    def __lt__(self, other):
        # Define less-than comparison to use the node with the lowest total cost in priority queue
        return self.total_cost < other.total_cost

def is_valid(row, col, grid):
    # Check if a given position (row, col) is within the grid and corresponds to a valid (walkable) cell
    return 0 <= row < len(grid) and 0 <= col < len(grid[0]) and grid[row][col] == 0

def get_neighbors(node, grid):
    # Get neighboring positions of a given node in the grid
    neighbors = []
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]  # Possible movement directions: down, up, right, left

    for dr, dc in directions:
        new_row, new_col = node.row + dr, node.col + dc
        if is_valid(new_row, new_col, grid):  # Check if the new position is valid
            neighbors.append((new_row, new_col))

    return neighbors

def calculate_heuristic(row, col, goal):
    # Calculate the heuristic value (Manhattan distance) from a position (row, col) to the goal position
    return abs(row - goal[0]) + abs(col - goal[1])

def reconstruct_path(goal_node):
    # Reconstruct the path from the goal node to the start node by traversing through parent nodes
    path = []
    current = goal_node
    while current is not None:
        path.append((current.row, current.col))
        current = current.parent
    return path[::-1]  # Reverse the path to start from the start node

def astar(grid, start, goal):
    # A* algorithm implementation to find the shortest path from start to goal in the grid
    start_node = Node(start[0], start[1], 0, calculate_heuristic(start[0], start[1], goal))
    goal_node = Node(goal[0], goal[1], 0, 0)

    open_set = [start_node]  # Priority queue (heap) for open set
    closed_set = set()       # Set to store visited nodes

    while open_set:
        current_node = heapq.heappop(open_set)  # Pop the node with the lowest total cost from the open set

        if (current_node.row, current_node.col) == (goal_node.row, goal_node.col):
            # If the goal node is reached, reconstruct and return the path
            return reconstruct_path(current_node)

        closed_set.add((current_node.row, current_node.col))  # Add the current node to the closed set

        # Explore neighbors of the current node
        for neighbor_row, neighbor_col in get_neighbors(current_node, grid):
            if (neighbor_row, neighbor_col) in closed_set:
                continue  # Skip visited neighbors

            # Create a neighbor node and calculate its cost and heuristic
            neighbor_node = Node(neighbor_row, neighbor_col, current_node.cost + 1,
                                 calculate_heuristic(neighbor_row, neighbor_col, goal))

            if neighbor_node not in open_set:
                # If the neighbor is not in the open set, add it and set its parent to the current node
                neighbor_node.parent = current_node
                heapq.heappush(open_set, neighbor_node)

    return None  # No path found

def update_velocity(particle, master_path,i):
    global next_point, velocity
    # Update particle velocity to follow the path of the master robot
    # Here, we calculate the next point in the master's path that the particle should aim for
    # Then, we adjust the velocity towards that point
    #current_point_index = len(master_path)-1
    next_point = master_path[i]
    velocity = np.array([next_point[0] - particle["position"][0], next_point[1] - particle["position"][1]])
    print("next targeeted point: ",next_point,"velocity: ",velocity)

def move_robot(particle, velocity):
    # Move the robot based on the calculated velocity
    # Here, we simply add the velocity to the current position of the robot
    particle["position"] += velocity

# Example usage:
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0]
]

start_position_master = (0, 0)  # Starting position for the master robot
start_position_slave1 = (1, 0)  # Starting position for the first slave robot
start_position_slave2 = (2, 0)  # Starting position for the second slave robot
goal_position = (4, 4)          # Goal position for all robots

# A* algorithm for master robot
master_path = astar(grid, start_position_master, goal_position)

if master_path:
    print("Master Robot Path:", master_path)
else:
    print("Master Robot: No path found.")

# PSO algorithm for slave robots
num_robots = 2
i=0
particles = [{"position": np.array(start_position_slave1), "current_point_index": 0},
             {"position": np.array(start_position_slave2), "current_point_index": 0}]

# Simulate movement for a certain number of iterations
num_iterations = 9
for _ in range(num_iterations):
    #print(_)
    for particle in particles:
        # Update velocity using PSO algorithm to follow the master path
        update_velocity(particle, master_path,_)
        # Move the robot based on the calculated velocity
        move_robot(particle, velocity)
        

        # Print final positions of slave robots
        for i, particle in enumerate(particles, start=1):
            print(f"Slave Robot {i} Position:", particle["position"])
        print()
    
