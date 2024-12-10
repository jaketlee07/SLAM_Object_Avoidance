#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, Imu
from flask import Flask, jsonify
import threading
import socket
from pylimo import limo

app = Flask(__name__)

# Global variables to store latest data
latest_scan = None
latest_imu = None

# Initialize LIMO
limo_robot = limo.LIMO()
limo_robot.EnableCommand()

# LiDAR callback
def laser_callback(msg):
    global latest_scan
    latest_scan = msg

# IMU callback
def imu_callback(msg):
    global latest_imu
    latest_imu = msg

# Endpoint to get combined data
@app.route('/get_combined_data', methods=['GET'])
def get_combined_data():
    if latest_scan is not None and latest_imu is not None:
        return jsonify({
            'lidar': {
                'header': {
                    'seq': latest_scan.header.seq,
                    'stamp': {
                        'secs': latest_scan.header.stamp.secs,
                        'nsecs': latest_scan.header.stamp.nsecs
                    },
                    'frame_id': latest_scan.header.frame_id
                },
                'angle_min': latest_scan.angle_min,
                'angle_max': latest_scan.angle_max,
                'angle_increment': latest_scan.angle_increment,
                'time_increment': latest_scan.time_increment,
                'scan_time': latest_scan.scan_time,
                'range_min': latest_scan.range_min,
                'range_max': latest_scan.range_max,
                'ranges': latest_scan.ranges,
                'intensities': latest_scan.intensities
            },
            'imu': {
                'header': {
                    'seq': latest_imu.header.seq,
                    'stamp': {
                        'secs': latest_imu.header.stamp.secs,
                        'nsecs': latest_imu.header.stamp.nsecs
                    },
                    'frame_id': latest_imu.header.frame_id
                },
                'orientation': {
                    'x': latest_imu.orientation.x,
                    'y': latest_imu.orientation.y,
                    'z': latest_imu.orientation.z,
                    'w': latest_imu.orientation.w
                },
                'angular_velocity': {
                    'x': latest_imu.angular_velocity.x,
                    'y': latest_imu.angular_velocity.y,
                    'z': latest_imu.angular_velocity.z
                },
                'linear_acceleration': {
                    'x': latest_imu.linear_acceleration.x,
                    'y': latest_imu.linear_acceleration.y,
                    'z': latest_imu.linear_acceleration.z
                }
            }
        })
    else:
        return jsonify({'error': 'No data available'})

# Run Flask app in a separate thread
def run_flask():
    app.run(host='0.0.0.0', port=5000)

# Function to handle movement commands
def handle_movement_commands(host='0.0.0.0', port=12345):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print(f"Server listening for movement commands on {host}:{port}")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr}")

        while True:
            data = client_socket.recv(1024).decode('utf-8')
            print(f"Received data: {data}")

            try:
                linear_vel, steering_angle = map(float, data.split(','))
                maxSpeed = 0.2
                if linear_vel > maxSpeed:
                    linear_vel = maxSpeed

                response = f"Set motion command: linear_vel={linear_vel}, steering_angle={steering_angle}"
                print(f"Setting motion command: linear_vel={linear_vel}, angular_velocity={steering_angle}")

                limo_robot.SetMotionCommand(linear_vel=linear_vel, steering_angle=steering_angle)
                rospy.sleep(0.1)

            except ValueError:
                response = "Invalid command format. Expected format: 'linear_vel,steering_angle'"

            client_socket.send(response.encode('utf-8'))

        client_socket.close()

if __name__ == '__main__':
    rospy.init_node('lidar_imu_data_server')
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.Subscriber('/imu', Imu, imu_callback)

    flask_thread = threading.Thread(target=run_flask)
    flask_thread.start()

    movement_thread = threading.Thread(target=handle_movement_commands)
    movement_thread.start()

    rospy.spin()
