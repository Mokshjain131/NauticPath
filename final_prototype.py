import numpy as np
import heapq
import folium
from folium.plugins import AntPath
import math

def heuristic(a, b):
    """Calculate the diagonal distance heuristic."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return max(dx, dy)  # Chebyshev distance for diagonal movement

def a_star(start, end, land_matrix, wind_matrix, current_matrix):
    """Perform A* pathfinding algorithm with diagonal movement."""
    rows, cols = land_matrix.shape
    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, end), 0, start))
    
    came_from = {}
    g_score = np.full(land_matrix.shape, np.inf)
    g_score[start] = 0
    
    f_score = np.full(land_matrix.shape, np.inf)
    f_score[start] = heuristic(start, end)
    
    while open_list:
        _, current_g, current = heapq.heappop(open_list)
        
        if current == end:
            # Reconstruct the path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Return reversed path
        
        # Explore neighbors (including diagonals)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Check if the neighbor is within bounds
            if (0 <= neighbor[0] < rows) and (0 <= neighbor[1] < cols):
                if land_matrix[neighbor] == 1:  # Skip land cells
                    continue
                
                # Calculate costs
                wind_cost = wind_matrix[neighbor]  # Example cost from wind matrix
                current_cost = current_matrix[neighbor]  # Example cost from current matrix
                # Diagonal movement cost adjustment
                if dx != 0 and dy != 0:  # Diagonal move
                    tentative_g_score = g_score[current] + wind_cost + current_cost + math.sqrt(2)  # Add sqrt(2) for diagonal cost
                else:  # Straight move
                    tentative_g_score = g_score[current] + wind_cost + current_cost
                
                # Update path if a better score is found
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(open_list, (f_score[neighbor], tentative_g_score, neighbor))
    
    return []  # Return an empty path if no path is found

def bezier_curve(points, num_points=100):
    """Generate points along a cubic Bezier curve."""
    if len(points) != 4:
        raise ValueError("Bezier curve requires exactly 4 control points.")
    
    t = np.linspace(0, 1, num_points)
    curve = (1-t)**3 * np.array(points[0])[:, None] + \
            3 * (1-t)**2 * t * np.array(points[1])[:, None] + \
            3 * (1-t) * t**2 * np.array(points[2])[:, None] + \
            t**3 * np.array(points[3])[:, None]
    
    return curve.T  # Return as a list of points

# Example usage
try:
    land_matrix = np.load('land.npy')
    wind_matrix = np.load('wind_data.npy')  # Example wind cost data
    current_matrix = np.load('grid.npy')  # Load the binary land matrix
except FileNotFoundError as e:
    print(f"Error: {e}")
    exit()
except ValueError:
    print("Error: Invalid data in input files.")
    exit()

start = (18, 79)  # Starting coordinates
end = (6, 79)    # Ending coordinates
NaviMumbai_Port = [18.9311, 72.9586]  # Center for the map

# Create a map with Folium
mymap = folium.Map(location=NaviMumbai_Port, zoom_start=5)
folium.Marker(start, popup='Start Point').add_to(mymap)
folium.Marker(end, popup='Goal Point').add_to(mymap)

# Find the optimal path
path = a_star(start, end, land_matrix, wind_matrix, current_matrix)

# Add the Bezier curve to the path
if path:
    # Generate Bezier curve points
    if len(path) >= 4:
        # Use the first, middle, and last points as control points
        control_points = [path[0], path[len(path) // 2], path[len(path) // 2 + 1], path[-1]]
        bezier_points = bezier_curve(control_points)
        bezier_path = [(int(point[0]), int(point[1])) for point in bezier_points]  # Convert to integer coordinates
    else:
        bezier_path = path  # If not enough points, use the original path
    
    AntPath(locations=bezier_path, color='blue', weight=3, opacity=0.8, dash_array=[10, 20]).add_to(mymap)
else:
    print("No path found.")

# Save the map to an HTML file
mymap.save("mymap.html")