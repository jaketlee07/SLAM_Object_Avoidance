import requests
import time
import numpy as np
import g2o
import math
import matplotlib.pyplot as plt

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

def create_vertex_se2(id, x, y, theta):
    v = g2o.VertexSE2()
    v.set_id(id)
    v.set_estimate(g2o.SE2(x, y, theta))
    return v

def create_edge_se2(vertices, measurement, information):
    edge = g2o.EdgeSE2()
    for i, v in enumerate(vertices):
        edge.set_vertex(i, v)
    edge.set_measurement(g2o.SE2(*measurement))
    edge.set_information(information)
    return edge

def polar_to_cartesian(ranges, angles):
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    return x, y

def update_map(map_data, pose, ranges, angles):
    x, y = polar_to_cartesian(ranges, angles)
    x_global = x * math.cos(pose[2]) - y * math.sin(pose[2]) + pose[0]
    y_global = x * math.sin(pose[2]) + y * math.cos(pose[2]) + pose[1]
    map_data.extend(list(zip(x_global, y_global)))

def main():
    robot_ip = "192.168.0.195"
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
    solver = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(solver)

    vertex_id = 0
    first_vertex = create_vertex_se2(vertex_id, 0, 0, 0)
    optimizer.add_vertex(first_vertex)

    prev_pose = np.array([0, 0, 0])
    prev_time = time.time()

    map_data = []
    plt.figure(figsize=(10, 10))
    plt.ion()

    while True:
        lidar_data = get_lidar_data(robot_ip)
        if lidar_data:
            ranges = np.array(lidar_data['ranges'])
            angle_min = lidar_data['angle_min']
            angle_increment = lidar_data['angle_increment']
            angles = np.arange(angle_min, angle_min + len(ranges) * angle_increment, angle_increment)

            current_time = time.time()
            dt = current_time - prev_time
            dx = 0.1 * math.cos(prev_pose[2]) * dt
            dy = 0.1 * math.sin(prev_pose[2]) * dt
            dtheta = 0.05 * dt

            current_pose = prev_pose + np.array([dx, dy, dtheta])

            vertex_id += 1
            new_vertex = create_vertex_se2(vertex_id, *current_pose)
            optimizer.add_vertex(new_vertex)

            information = np.identity(3) * 100
            edge = create_edge_se2([optimizer.vertex(vertex_id-1), new_vertex], 
                                   [dx, dy, dtheta], information)
            optimizer.add_edge(edge)

            optimizer.initialize_optimization()
            optimizer.optimize(10)

            optimized_pose = optimizer.vertex(vertex_id).estimate().to_vector()
            print(f"Optimized pose: {optimized_pose}")

            update_map(map_data, optimized_pose, ranges, angles)

            plt.clf()
            map_array = np.array(map_data)
            plt.scatter(map_array[:, 0], map_array[:, 1], s=1, c='b', alpha=0.5)
            plt.plot(optimized_pose[0], optimized_pose[1], 'ro')
            plt.arrow(optimized_pose[0], optimized_pose[1], 
                      0.5*math.cos(optimized_pose[2]), 0.5*math.sin(optimized_pose[2]), 
                      head_width=0.2, head_length=0.3, fc='r', ec='r')
            plt.xlim(-10, 10)
            plt.ylim(-10, 10)
            plt.title("SLAM Map")
            plt.draw()
            plt.pause(0.001)

            prev_pose = optimized_pose
            prev_time = current_time

        time.sleep(0.1)

if __name__ == "__main__":
    main()
