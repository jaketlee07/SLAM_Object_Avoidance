import numpy as np
import math
import matplotlib.pyplot as plt
import threading
import time
import queue

class LidarSLAM:
    def __init__(self, robot_ip, map_size=50, map_resolution=0.05):
        """
        Initialize SLAM mapping system
        
        :param robot_ip: IP address of the Agilex Limo
        :param map_size: Size of the map in meters
        :param map_resolution: Resolution of the occupancy grid (meters per cell)
        """
        # Map parameters
        self.map_size = map_size
        self.map_resolution = map_resolution
        
        # Occupancy grid initialization
        self.grid_size = int(map_size / map_resolution)
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)
        
        # Robot connection parameters
        self.robot_ip = robot_ip
        
        # Robot state tracking
        self.current_pose = [0, 0, 0]  # x, y, theta
        
        # Thread-safe visualization
        self.grid_queue = queue.Queue()
        self.visualization_thread = None
        self.stop_visualization = False
        
        # Mapping thread
        self.mapping_thread = None
        self.stop_mapping = False
        
        # Start visualization thread
        self.start_visualization()
    
    def get_lidar_data(self):
        """
        Fetch LiDAR data from the robot
        
        :return: Dictionary containing LiDAR scan data
        """
        import requests
        
        url = f"http://{self.robot_ip}:5000/get_scan"
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
    
    def convert_scan_to_points(self, scan_data):
        """
        Convert LiDAR scan data to Cartesian points
        
        :param scan_data: Dictionary containing ranges and other scan parameters
        :return: List of (x, y) points
        """
        if not scan_data or 'ranges' not in scan_data:
            return []
        
        ranges = scan_data['ranges']
        points = []
        
        # Scan parameters from the provided data
        angle_min = scan_data.get('angle_min', -1.57079637051)
        angle_increment = scan_data.get('angle_increment', 0.00715624773875)
        range_min = scan_data.get('range_min', 0.1)
        range_max = scan_data.get('range_max', 12.0)
        
        for i, distance in enumerate(ranges):
            # Filter out invalid measurements
            if distance > range_min and distance < range_max:
                angle = angle_min + i * angle_increment
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                points.append((x, y))
        
        return points
    
    def update_occupancy_grid(self, points):
        """
        Update occupancy grid based on LiDAR points
        
        :param points: List of (x, y) points from LiDAR scan
        """
        # Create a copy of the current grid to avoid race conditions
        current_grid = self.occupancy_grid.copy()
        
        for x, y in points:
            # Transform point to world coordinates (assuming robot at origin)
            wx = x * math.cos(self.current_pose[2]) - y * math.sin(self.current_pose[2]) + self.current_pose[0]
            wy = x * math.sin(self.current_pose[2]) + y * math.cos(self.current_pose[2]) + self.current_pose[1]
            
            # Convert to grid coordinates
            gx = int((wx + self.map_size/2) / self.map_resolution)
            gy = int((wy + self.map_size/2) / self.map_resolution)
            
            # Bounds check
            if (0 <= gx < self.grid_size and 0 <= gy < self.grid_size):
                current_grid[gy, gx] = 100  # Mark as occupied
        
        # Update the grid atomically
        self.occupancy_grid = current_grid
        
        # Put grid in queue for visualization
        self.grid_queue.put(current_grid)
    
    def visualization_loop(self):
        """
        Separate thread for visualization
        """
        import matplotlib.pyplot as plt
        import queue

        plt.ion()
        fig, ax = plt.subplots()
        
        while not self.stop_visualization:
            try:
                # Try to get the latest grid with a timeout
                grid = self.grid_queue.get(timeout=0.5)
                
                # Use plt.pause() from the main thread
                plt.figure(fig.number)
                ax.clear()
                ax.imshow(grid, cmap='gray_r', origin='lower')
                ax.set_title('Real-time Occupancy Grid')
                fig.canvas.draw()
                fig.canvas.flush_events()
                time.sleep(0.1)
            
            except queue.Empty:
                # No new grid, just continue
                time.sleep(0.1)
        
        plt.close(fig)
    
    def start_visualization(self):
        """
        Start the visualization thread
        """
        self.stop_visualization = False
        self.visualization_thread = threading.Thread(target=self.visualization_loop)
        self.visualization_thread.start()
    
    def run_mapping(self, update_interval=1):
        """
        Continuous mapping thread
        
        :param update_interval: Time between LiDAR scans (seconds)
        """
        while not self.stop_mapping:
            # Fetch LiDAR data
            lidar_data = self.get_lidar_data()
            
            if lidar_data:
                # Convert scan to points
                points = self.convert_scan_to_points(lidar_data)
                
                # Update occupancy grid
                self.update_occupancy_grid(points)
            
            # Wait before next scan
            time.sleep(update_interval)
    
    def start_mapping(self):
        """
        Start the mapping process in a separate thread
        """
        self.stop_mapping = False
        self.mapping_thread = threading.Thread(target=self.run_mapping)
        self.mapping_thread.start()
    
    def stop_mapping_process(self):
        """
        Stop the mapping process
        """
        self.stop_mapping = True
        self.stop_visualization = True
        
        if self.mapping_thread:
            self.mapping_thread.join()
        
        if self.visualization_thread:
            self.visualization_thread.join()

def main():
    """
    Main function to demonstrate LiDAR SLAM mapping
    """
    # Replace with your Agilex Limo's IP address
    robot_ip = "192.168.0.195"
    
    # Create SLAM instance
    slam = LidarSLAM(robot_ip)
    
    try:
        # Start mapping
        slam.start_mapping()
        
        # Keep main thread running
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        # Clean up on interrupt
        print("\nStopping mapping process...")
        slam.stop_mapping_process()

if __name__ == "__main__":
    main()