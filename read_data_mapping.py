import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import requests
import time

def get_lidar_data(robot_ip, port=5000):
    url = f"http://{robot_ip}:{port}/get_scan"
    try:
        response = requests.get(url)
        if response.status_code == 200:
            return response.json()
        else:
            print(f"Error: Received status code {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")
        return None

def create_map_from_lidar(data):
    if data is None:
        print("No data received")
        return None, None

    ranges = np.array(data['ranges'])
    angle_min = data['angle_min']
    angle_increment = data['angle_increment']

    valid_indices = np.where((ranges > data['range_min']) & (ranges < data['range_max']))
    ranges = ranges[valid_indices]
    angles = angle_min + valid_indices[0] * angle_increment

    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    return x, y

def create_occupancy_grid(x, y, map_size=100, resolution=0.1):
    occupancy_grid = np.zeros((map_size, map_size))
    x_indices = (x / resolution + map_size // 2).astype(int)
    y_indices = (y / resolution + map_size // 2).astype(int)

    for xi, yi in zip(x_indices, y_indices):
        if 0 <= xi < map_size and 0 <= yi < map_size:
            occupancy_grid[yi, xi] = 1

    return occupancy_grid

# Set up the figure and subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
scatter = ax1.scatter([], [], s=5, c='blue', label='LiDAR Points')
occupancy_image = ax2.imshow(np.zeros((100, 100)), cmap='Greys', origin='lower')

ax1.set_xlim(-5, 5)
ax1.set_ylim(-5, 5)
ax1.set_xlabel('X (meters)')
ax1.set_ylabel('Y (meters)')
ax1.set_title('2D Map of Surroundings')
ax1.legend()
ax1.grid(True)

ax2.set_title('2D Occupancy Grid Map')
ax2.set_xlabel('X (grid cells)')
ax2.set_ylabel('Y (grid cells)')

robot_ip = "192.168.0.195"  # Replace with your robot's IP address

def update(frame):
    lidar_data = get_lidar_data(robot_ip)
    
    if lidar_data:
        x, y = create_map_from_lidar(lidar_data)
        if x is not None and y is not None:
            scatter.set_offsets(np.column_stack((x, y)))
            
            occupancy_grid = create_occupancy_grid(x, y)
            occupancy_image.set_array(occupancy_grid)
    
    return scatter, occupancy_image

# Create the animation
ani = FuncAnimation(fig, update, frames=None, interval=1000, blit=True)

plt.tight_layout()
plt.show()
