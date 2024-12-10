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
if __name__ == "__main__":
    robot_ip = "192.168.5.205"  # Replace with your robot's IP address
    
    while True:
        lidar_data = get_lidar_data(robot_ip)
        print(lidar_data)
        time.sleep(1)  # Adjust the delay as needed