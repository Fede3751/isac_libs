import math
import numpy as np


def edge_device_placement(n, field_width, field_height, sensor_range):

    #sensor_range = 50

    if field_width < sensor_range * 2 or field_height < sensor_range * 2:
         raise ValueError("Field dimensions are too small for the sensor range.")

    center_x = 0 #field_width // 2
    center_y = 0 #field_height // 2

    # If there is only one edge device, place it at the center of the field
    if n == 1:
        return [(center_x, center_y, 0)]

    # Calculate the radius of the circumcircle for the regular polygon
    circumcircle_radius = sensor_range / (2 * math.sin(math.pi / n))

    # Calculate the positions of the edge devices at the vertices of the regular polygon
    edge_devices_coords = []

    for i in range(n):
        angle = 2 * math.pi * i / n
        bx = round(center_x + circumcircle_radius * math.cos(angle))
        by = round(center_y + circumcircle_radius * math.sin(angle))

        edge_devices_coords.append((bx, by, 1.0))

    return edge_devices_coords


def compute_area_coverage(field_width, field_height, edge_devices_coords, sensor_range):
        

        #sensor_range = 50

        
        # Generate a grid of points across the field
        x_points = np.linspace(0, field_width, 500)
        y_points = np.linspace(0, field_height, 500)
        grid_x, grid_y = np.meshgrid(x_points, y_points)

        # Initialize a grid to track coverage
        coverage_grid = np.zeros_like(grid_x, dtype=bool)

        # Check if each point in the grid is within the range of any edge device
        for (bx, by, _) in edge_devices_coords:
            coverage_grid |= (grid_x - bx)**2 + (grid_y - by)**2 <= sensor_range**2
        
        return coverage_grid, grid_x, grid_y


def compute_random_covered_positions(n, coverage_grid, grid_x, grid_y):
    sensor_coords = []
    sensor_indices = np.argwhere(coverage_grid)

    if len(sensor_indices) < n:
        raise ValueError("Not enough coverage to place all sensors.")

    # Randomly select n sensor positions from the covered area
    selected_indices = np.random.choice(len(sensor_indices), n, replace=False)
    for idx in selected_indices:
        sensor_x, sensor_y = sensor_indices[idx]
        # Round the coordinates
        rounded_x = round(grid_x[sensor_x, sensor_y])
        rounded_y = round(grid_y[sensor_x, sensor_y])
        sensor_coords.append((rounded_x, rounded_y, 0.0))

    return sensor_coords


def base_station_placement(field_width, field_height, offset=50, outside=True):
    # Place the base station at the center of the field
    bs_x = field_width // 2
    bs_y = field_height // 2
    bs_z = 0

    # Place the base station outside the field by the specified offset        
    if outside:
        bs_x = field_width + offset
        bs_y = field_height + offset

    return (bs_x, bs_y, bs_z)


