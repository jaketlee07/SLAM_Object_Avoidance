# #!/usr/bin/env python3
# import rospy
# from sensor_msgs.msg import LaserScan
# from flask import Flask, jsonify
# import threading

# app = Flask(__name__)

# # Global variable to store the latest scan data
# latest_scan = None

# def laser_callback(msg):
#     global latest_scan
#     latest_scan = msg

# @app.route('/get_scan', methods=['GET'])
# def get_scan():
#     if latest_scan is not None:
#         return jsonify({
#             'header': {
#                 'seq': latest_scan.header.seq,
#                 'stamp': {
#                     'secs': latest_scan.header.stamp.secs,
#                     'nsecs': latest_scan.header.stamp.nsecs
#                 },
#                 'frame_id': latest_scan.header.frame_id
#             },
#             'angle_min': latest_scan.angle_min,
#             'angle_max': latest_scan.angle_max,
#             'angle_increment': latest_scan.angle_increment,
#             'time_increment': latest_scan.time_increment,
#             'scan_time': latest_scan.scan_time,
#             'range_min': latest_scan.range_min,
#             'range_max': latest_scan.range_max,
#             'ranges': latest_scan.ranges,
#             'intensities': latest_scan.intensities
#         })
#     else:
#         return jsonify({'error': 'No scan data available'})

# def run_flask():
#     app.run(host='0.0.0.0', port=5000)

# if __name__ == '__main__':
#     rospy.init_node('lidar_data_server')
#     rospy.Subscriber('/scan', LaserScan, laser_callback)
    
#     # Start Flask in a separate thread
#     flask_thread = threading.Thread(target=run_flask)
#     flask_thread.start()
    
#     rospy.spin()

from sensor_msgs.msg import LaserScan, Imu
from flask import Flask, jsonify
import threading

app = Flask(__name__)

# Global variables to store latest data
latest_scan = None
latest_imu = None

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

if __name__ == '__main__':
    rospy.init_node('lidar_imu_data_server')
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.Subscriber('/imu', Imu, imu_callback)

    flask_thread = threading.Thread(target=run_flask)
    flask_thread.start()

    rospy.spin()
