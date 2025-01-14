#! /usr/bin/env python

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Point, Quaternion
from demo01.msg import RobotState, RobotPath, RobotDimension
import heapq
import math


# ESDF MAP
esdf_map = [
    [3, 3, 2, 2, 1, 0, 1],
    [3, 3, 3, 2, 1, 0, 1],
    [3, 3, 3, 2, 1, 1, 2],
    [3, 3, 3, 3, 2, 2, 3],
    [3, 3, 3, 3, 3, 3, 3]
]

# Range setting
MIN_WIDTH, MAX_WIDTH = 1, 3
MIN_LENGTH, MAX_LENGTH = 1, 3
MAX_ESDF = max(max(row) for row in esdf_map)  # Max ESDF value

# 
class Node:
    def __init__(self, x, y, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.g = g   
        self.h = h   
        self.f = g + h                                                                                                                              
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

# Adjust robot size according to ESDF value
def adjust_robot_size(esdf_value):
    width = MIN_WIDTH + (1 - esdf_value / MAX_ESDF) * (MAX_WIDTH - MIN_WIDTH)
    length = MIN_LENGTH + (esdf_value / MAX_ESDF) * (MAX_LENGTH - MIN_LENGTH)
    return width, length

# Collision detection
def is_collision_free(x, y, width, length, esdf_map):
    rows, cols = len(esdf_map), len(esdf_map[0])
    # Boundary detection
    if x < 0 or x >= rows or y < 0 or y >= cols:
        return False
    # Safety detection: ESDF and robot size
    return esdf_map[x][y] >= max(width / 2, length / 2)

def astar(esdf_map, start, goal):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    rows, cols = len(esdf_map), len(esdf_map[0])

    open_list = []
    closed_list = set()

    node_sizes = {} 

    start_node = Node(start[0], start[1], 0, heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    start_esdf_value = esdf_map[start[0]][start[1]]
    node_sizes[start] = adjust_robot_size(start_esdf_value) 

    while open_list:
        current_node = heapq.heappop(open_list)

        if (current_node.x, current_node.y) == goal:
            path = reconstruct_path(current_node)
            return path, node_sizes

        closed_list.add((current_node.x, current_node.y))

        for dx, dy in directions:
            nx, ny = current_node.x + dx, current_node.y + dy

            if 0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in closed_list:
                esdf_value = esdf_map[nx][ny]
                width, length = adjust_robot_size(esdf_value)

                if not is_collision_free(nx, ny, width, length, esdf_map):
                    continue

                g_cost = current_node.g + 1  # default edge cost 1
                h_cost = heuristic((nx, ny), goal)
                neighbor_node = Node(nx, ny, g_cost, h_cost, current_node)

                if not any(neighbor.x == nx and neighbor.y == ny and neighbor.f <= neighbor_node.f for neighbor in open_list):
                    heapq.heappush(open_list, neighbor_node)
                    node_sizes[(nx, ny)] = (width, length)  # current node size

    return None, None


def heuristic(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def reconstruct_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]






if __name__ == "__main__":

    rospy.init_node("publish_node")
    rate = rospy.Rate(1)
    # demo
    start = (0, 0)
    goal = (4, 6)
    # rospy.loginfo(goal[0])
    path, node_sizes = astar(esdf_map, start, goal)
    # rospy.loginfo(node_sizes)
    robot_path = RobotPath()

    states = []

    if path:
        for node in path:
            state = RobotState()

            state.header = Header()
            state.header.stamp = rospy.Time.now()
            state.header.frame_id = "ESDF_MAP"
            state.position = Point(node[0], node[1], 0)
            state.orientation = Quaternion(0, 0, 0, 1)
            state.dimension.length = format(node_sizes[node][1], '.2f')
            state.dimension.width = format(node_sizes[node][0], '.2f')
            
            states.append(state)

            print(f"Position {node}: Width = {node_sizes[node][0]:.2f}, Length = {node_sizes[node][1]:.2f}")
    else:
        rospy.loginfo("NO PATH FOUNDED")

    robot_path.states = states
    pub = rospy.Publisher("Robot_Path", RobotPath, queue_size=10)
    rospy.loginfo(robot_path)

    while not rospy.is_shutdown():
        pub.publish(robot_path)
        
        rate.sleep()

    
