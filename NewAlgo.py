import geopandas as gpd
import numpy as np
import networkx as nx
from shapely.geometry import Point

shapefile_path = 'C:\Users\moksh\OneDrive\Desktop\ShipLoc\SIMROUTE-master\shiploc\ne_50m_admin_0_countries.shp'

# Step 1: Load global landmass data (using a GeoJSON file)
world = gpd.read_file(gpd.datasets.get_path('naturalearth_lowres'))

# Step 2: Define function to check if a point is on land or sea
def is_on_land(point, world_df):
    return world_df.contains(point).any()

# Step 3: Create a grid of waypoints for navigation (lat/lon grid)
def create_waypoint_grid(lat_range, lon_range, resolution=1.0):
    lats = np.arange(lat_range[0], lat_range[1], resolution)
    lons = np.arange(lon_range[0], lon_range[1], resolution)
    waypoints = [Point(lon, lat) for lat in lats for lon in lons]
    return waypoints

# Step 4: Create graph for route calculation (only sea points)
def create_graph(waypoints, world_df):
    G = nx.Graph()
    for i, wp in enumerate(waypoints):
        if not is_on_land(wp, world_df):
            G.add_node(i, pos=(wp.x, wp.y))
            # Add edges between neighboring waypoints
            for j in [i-1, i+1, i-len(waypoints), i+len(waypoints)]:
                if j >= 0 and j < len(waypoints) and not is_on_land(waypoints[j], world_df):
                    G.add_edge(i, j, weight=1)  # Simple distance metric
    return G

# Step 5: Calculate optimal route using Dijkstra's algorithm
def calculate_optimal_route(G, start_point, end_point):
    start_node = min(G.nodes, key=lambda n: (G.nodes[n]['pos'][0] - start_point.x)**2 + (G.nodes[n]['pos'][1] - start_point.y)**2)
    end_node = min(G.nodes, key=lambda n: (G.nodes[n]['pos'][0] - end_point.x)**2 + (G.nodes[n]['pos'][1] - end_point.y)**2)
    
    path = nx.dijkstra_path(G, start_node, end_node)
    route = [G.nodes[p]['pos'] for p in path]
    return route

# Step 6: Define the navigation route and visualize it
import matplotlib.pyplot as plt

def plot_route(route):
    lats, lons = zip(*route)
    plt.plot(lons, lats, 'bo-', markersize=3)
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Optimal Ship Route')
    plt.show()

# Example usage:
lat_range = (-90, 90)  # Latitude range for global coverage
lon_range = (-180, 180)  # Longitude range for global coverage

waypoints = create_waypoint_grid(lat_range, lon_range, resolution=5.0)  # Coarse grid (5-degree resolution)
G = create_graph(waypoints, world)

start = Point(-70.0, 40.0)  # Example start (near New York)
end = Point(139.0, 35.0)  # Example end (near Tokyo)

route = calculate_optimal_route(G, start, end)
plot_route(route)
