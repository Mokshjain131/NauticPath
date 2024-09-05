import heapq
import math

# Define a Point class to represent waypoints on the route
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def distance_to(self, other):
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

# Define a Node class to be used in the A* search
class Node:
    def __init__(self, point, g_cost=0, h_cost=0, parent=None):
        self.point = point
        self.g_cost = g_cost  # Cost from start to this node
        self.h_cost = h_cost  # Heuristic cost to the goal
        self.parent = parent
    
    @property
    def f_cost(self):
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        return self.f_cost < other.f_cost

# A* search algorithm for optimal routing
def a_star_search(start, goal, grid, wind_data, current_data):
    open_list = []
    heapq.heappush(open_list, Node(start, h_cost=start.distance_to(goal)))
    closed_list = set()
    
    while open_list:
        current_node = heapq.heappop(open_list)
        
        if current_node.point == goal:
            return reconstruct_path(current_node)
        
        closed_list.add(current_node.point)
        
        for neighbor in get_neighbors(current_node.point, grid, wind_data, current_data):
            if neighbor in closed_list:
                continue
            
            tentative_g_cost = current_node.g_cost + current_node.point.distance_to(neighbor)
            node = Node(neighbor, g_cost=tentative_g_cost, h_cost=neighbor.distance_to(goal), parent=current_node)
            
            heapq.heappush(open_list, node)
    
    return None  # No path found

# Function to get valid neighbors (simulating ocean grid)
def get_neighbors(point, grid, wind_data, current_data):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    neighbors = []
    
    for dx, dy in directions:
        new_x, new_y = point.x + dx, point.y + dy
        
        if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and grid[new_x][new_y] == 0:
            # Adjust for wind and current impacts
            adjusted_x = new_x + wind_data[new_x][new_y]
            adjusted_y = new_y + current_data[new_x][new_y]


            # Ensure the adjusted points are within bounds
            if 0 <= adjusted_x < len(grid) and 0 <= adjusted_y < len(grid[0]):
                neighbors.append(Point(adjusted_x, adjusted_y))
            else:
                neighbors.append(Point(new_x, new_y))  # Revert to unadjusted point if out of bounds
        else:
            continue
    
    return neighbors

# Reconstruct path from start to goal
def reconstruct_path(node):
    path = []
    while node:
        path.append((node.point.x, node.point.y))
        node = node.parent
    return path[::-1]

import numpy as np

grid = np.load('C:/Users/moksh/OneDrive/Desktop/ShipLoc/SIMROUTE-master/shiploc/grid_data.npy')

current_data = np.load('C:/Users/moksh/OneDrive/Desktop/ShipLoc/SIMROUTE-master/shiploc/current_data.npy')

wind_data = np.load('C:/Users/moksh/OneDrive/Desktop/ShipLoc/SIMROUTE-master/shiploc/wind_data.npy')

# Define start and goal points
start_point = Point(18, 72)
goal_point = Point(7, 80)

# Run the A* search algorithm
optimal_path = a_star_search(start_point, goal_point, grid, wind_data, current_data)

print("Optimal Path:", optimal_path)

