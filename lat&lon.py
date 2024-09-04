import numpy as np
import pandas as pd

# Define grid resolution
lat_resolution = 1.0  # 1 degree
lon_resolution = 1.0  # 1 degree

# Generate latitude and longitude arrays
latitudes = np.arange(-90, 90 + lat_resolution, lat_resolution)
longitudes = np.arange(-180, 180 + lon_resolution, lon_resolution)

# Create a meshgrid
lon_grid, lat_grid = np.meshgrid(longitudes, latitudes)

# Flatten the meshgrid to create a DataFrame
grid_data = pd.DataFrame({
    'latitude': lat_grid.flatten(),
    'longitude': lon_grid.flatten(),
})

# Display the first few rows of the grid data
print(grid_data.head())

# Optionally, save the grid data to a CSV file
grid_data.to_csv('global_grid.csv', index=False)
