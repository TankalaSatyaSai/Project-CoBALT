#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

class RRTStar:
    def __init__(self, start, goal, search_area, max_iter=1000, step_size=5.0):
        self.start = start
        self.goal = goal
        self.min_x, self.max_x, self.min_y, self.max_y = search_area
        self.max_iter = max_iter
        self.step_size = step_size
        self.node_list = [start]
        self.map_data = None

        
        rospy.init_node('rrtstar_planner')
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.wait_for_message('/map', OccupancyGrid)

    def map_callback(self, data):
        
        self.map_data = data

    def planning(self):
        for _ in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.nearest_node_index(rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node)
            if self.check_collision(new_node):
                near_nodes = self.find_near_nodes(new_node)
                self.choose_parent(new_node, near_nodes)
                self.node_list.append(new_node)

        path = self.generate_final_course()
        return path

    def steer(self, from_node, to_node):
        delta_x, delta_y = to_node[0] - from_node[0], to_node[1] - from_node[1]
        distance = np.hypot(delta_x, delta_y)
        if distance > self.step_size:
            scale = self.step_size / distance
            new_x = from_node[0] + delta_x * scale
            new_y = from_node[1] + delta_y * scale
            return [new_x, new_y]
        return to_node

    def get_random_node(self):
        x = np.random.uniform(self.min_x, self.max_x)
        y = np.random.uniform(self.min_y, self.max_y)
        return [x, y]

    def nearest_node_index(self, rnd_node):
        distances = [np.linalg.norm(np.array(node[:2]) - np.array(rnd_node)) for node in self.node_list]
        return np.argmin(distances)


    def choose_parent(self, new_node, near_nodes):
        if not near_nodes:
            return
        costs = [np.linalg.norm(np.array(node[:2]) - np.array(new_node)) for node in near_nodes]
        min_cost_idx = np.argmin(costs)
        new_node_cost = costs[min_cost_idx] + near_nodes[min_cost_idx][2]
        if new_node_cost < new_node[2]:
            new_node[:] = near_nodes[min_cost_idx][:2] + [new_node_cost]

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list)
        r = min(30.0 * np.sqrt((np.log(nnode) / nnode)), self.step_size * 5)
        return [node for node in self.node_list if np.linalg.norm(np.array(node[:2]) - np.array(new_node)) <= r]

    def generate_final_course(self):
        last_index = len(self.node_list) - 1
        path = [self.goal[:2]]  
        while last_index != 0:
            path.append(self.node_list[last_index][:2])  
            last_index = int(self.node_list[last_index][3])
        path.append(self.start[:2]) 
        return path[::-1]


    def check_collision(self, node):
        if self.map_data is None:
            return True  
        map_width = self.map_data.info.width
        map_height = self.map_data.info.height
        map_resolution = self.map_data.info.resolution
        map_origin = self.map_data.info.origin.position

        grid_x = int((node[0] - map_origin.x) / map_resolution)
        grid_y = int((node[1] - map_origin.y) / map_resolution)

        if grid_x < 0 or grid_x >= map_width or grid_y < 0 or grid_y >= map_height:
            return False  

        grid_value = self.map_data.data[grid_y * map_width + grid_x]
        return grid_value > 50  

def main():
    start = [10, 10, 0]  
    goal = [300, 200]
    search_area = (0, 400, 0, 300)  

    rrt_star = RRTStar(start, goal, search_area)
    path = rrt_star.planning()

    if path:
        print("Found path:")
        for i, point in enumerate(path):
            print(f"Waypoint {i + 1}: X: {point[0]}, Y: {point[1]}")
    else:
        print("No valid path found.")

    plt.figure(figsize=(8, 6))
    plt.plot(start[0], start[1], 'ro', markersize=8)
    plt.plot(goal[0], goal[1], 'go', markersize=8)
    if path:
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
