import requests
import time

def get_lidar_data(robot_ip, port=500):
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

def process_lidar_data(data):
    if data is None:
        return
    
    # Example: Print some basic information
    print(f"Sequence: {data['header']['seq']}")
    print(f"Timestamp: {data['header']['stamp']['secs']}.{data['header']['stamp']['nsecs']}")
    print(f"Number of range readings: {len(data['ranges'])}")
    print(f"Min range: {min(filter(lambda x: x > 0, data['ranges']))}")
    print(f"Max range: {max(data['ranges'])}")
    print("---")

if __name__ == "__main__":
    robot_ip = "192.168.0.195"  # Replace with your robot's IP address
    
    while True:
        lidar_data = get_lidar_data(robot_ip)
        process_lidar_data(lidar_data)
        time.sleep(1)  # Adjust the delay as needed