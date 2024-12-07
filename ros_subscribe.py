#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from flask import Flask, jsonify
import threading

app = Flask(__name__)

# Global variable to store the latest scan data
latest_scan = None

def laser_callback(msg):
    global latest_scan
    latest_scan = msg

@app.route('/get_scan', methods=['GET'])
def get_scan():
    if latest_scan is not None:
        return jsonify({
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
        })
    else:
        return jsonify({'error': 'No scan data available'})

def run_flask():
    app.run(host='0.0.0.0', port=11311)

if __name__ == '__main__':
    rospy.init_node('lidar_data_server')
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    
    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.start()
    
    rospy.spin()