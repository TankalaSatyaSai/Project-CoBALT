#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import heapq

class AStarPlanner:
    def __init__(self):
        rospy.init_node('a_star_planner')

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        
        self.path_pub = rospy.Publisher('/planned_path', PoseStamped, queue_size=10)

        self.map_info = None
        self.grid = None

        rospy.spin()

    def map_callback(self, msg):
        self.map_info = msg.info
        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        start = (10, 10)
        goal = (50, 50)

        
        path = self.a_star(start, goal)
        self.print_waypoints(path)

        
        self.publish_path(path)

    def a_star(self, start, goal):
        def heuristic_cost_estimate(node, goal):
            return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

        def reconstruct_path(came_from, current):
            total_path = [current]
            while current in came_from:
                current = came_from[current]
                total_path.append(current)
            return total_path[::-1]

        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while frontier:
            current_cost, current_node = heapq.heappop(frontier)

            if current_node == goal:
                return reconstruct_path(came_from, current_node)

            for next_node in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                neighbor = (current_node[0] + next_node[0], current_node[1] + next_node[1])

                if not self.is_valid(neighbor):
                    continue

                new_cost = cost_so_far[current_node] + 1  

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic_cost_estimate(neighbor, goal)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current_node

        return None  # No path found

    def is_valid(self, node):
        x, y = node
        return 0 <= x < self.map_info.width and 0 <= y < self.map_info.height and self.grid[y][x] < 50

    def publish_path(self, path):
        if path:
            path_msg = PoseStamped()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = rospy.Time.now()  

            for point in path:
                pose = Pose() 
                pose.position.x = point[0]
                pose.position.y = point[1]
                pose.orientation.w = 1.0  

                
                path_msg.pose = pose

                
                self.path_pub.publish(path_msg)
                #rospy.sleep(0.1) 
    
    def print_waypoints(self, path):
        if path:
            print("Waypoints:")
            for i, point in enumerate(path):
                print(f"Waypoint {i + 1}: X: {point[0]}, Y: {point[1]}")
        else:
            print("No valid path found.")

if __name__ == '__main__':
    try:
        AStarPlanner()
    except rospy.ROSInterruptException:
        pass
