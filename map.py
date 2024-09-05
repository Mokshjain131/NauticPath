import folium
from folium.plugins import AntPath
from geopy.distance import geodesic

NaviMumbai_Port = [18.9311, 72.9586]
my_map = folium.Map(location=NaviMumbai_Port, zoom_start=5)

Marker_NaviMumbai_Port = [18.943976, 72.940790]
Marker_Colombo_Port = [6.944845, 79.845143]

folium.Marker(Marker_NaviMumbai_Port, popup='Mumbai Port').add_to(my_map)
folium.Marker(Marker_Colombo_Port, popup='Colombo Port').add_to(my_map)

ocean_point1 = [18.883499, 72.852273]
ocean_point2 = [18.727865, 72.680095]
ocean_point3 = [15.838610, 73.109814]
ocean_point4 = [7.526654, 77.084407]
ocean_point5 = [6.967967, 79.846338]

line_points = [
    Marker_NaviMumbai_Port,
    ocean_point1,
    ocean_point2,
    ocean_point3,
    ocean_point4,
    ocean_point5,
    Marker_Colombo_Port]

total_distance = 0
for i in range(len(line_points) - 1):
    total_distance += geodesic(line_points[i], line_points[i+1]).km


AntPath(locations=line_points, color='blue', weight=3, opacity=0.8, dash_array=[10, 20]).add_to(my_map)

distance_marker_location = line_points[-1]  
folium.Marker(location=distance_marker_location, 
              popup=f"Total Path Distance: {total_distance:.2f} km", 
              icon=folium.Icon(color="red", icon="info-sign")).add_to(my_map)

my_map.save("my_map.html")
