#!/usr/bin/env python3
# coding=utf-8
from pylimo import limo
import time
import socket

limo = limo.LIMO()
limo.EnableCommand()

def start_server(host='0.0.0.0', port=12345):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print(f"Server listening on {host}:{port}")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr}")

        try:
            while True:
                data = client_socket.recv(1024).decode('utf-8')
                if not data:
                    break
                print(f"Received data: {data}")

                try:
                    linear_vel, steering_angle = map(float, data.split(','))
                    maxSpeed = 0.2
                    linear_vel = min(linear_vel, maxSpeed)

                    response = f"Set motion command: linear_vel={linear_vel}, steering_angle={steering_angle}"
                    print(f"Setting motion command: linear_vel={linear_vel}, steering_angle={steering_angle}")

                    limo.SetMotionCommand(linear_vel=linear_vel, steering_angle=steering_angle)
                    time.sleep(0.1)

                except ValueError:
                    response = "Invalid command format. Expected format: 'linear_vel,steering_angle'"

                client_socket.send(response.encode('utf-8'))
        except Exception as e:
            print(f"Error handling client: {e}")
        finally:
            client_socket.close()

if __name__ == "__main__":
    start_server()
