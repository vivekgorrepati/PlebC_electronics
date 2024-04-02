import threading
from queue import Queue
import PlebCEngine
import SocketCode
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import serial
import math as m

from datetime import datetime

# Define the serial port and baud rate
serial_port = 'COM3'  # Replace with your Arduino's serial port
baud_rate = 115200  # Replace with your Arduino's baud rate

# Create the serial connection
imu_ser = serial.Serial(serial_port, baud_rate)

websocket_url = "wss://torus-server-thxfxagavq-uc.a.run.app/"  # Replace with your WebSocket server URL
client = SocketCode.WebSocketClient(websocket_url)
client.start()

# Define a stack to store IMU data
imu_data_stack = []

# Create a lock to ensure thread-safe access to the stack
stack_lock = threading.Lock()

timeelapsed = 0
bestangles = [0, 0, 0]  # Initialize with a default value

prevq1 = 0
prevq3 = 0
prevq5 = 0

# Capture the real initial orientation from the IMU
initialRealOrientation = R.from_quat([1, 0, 0, 0])  # Actual IMU reading
desired_zero_orientation = R.from_quat([0, 0, 1, 0])  # Desired 'zero' orientation

# Function to read IMU data and store it in the stack
def imu_data_thread():
    global imu_data_stack
    global stack_lock
    

    while True:
        imu_data = imu_ser.readline().decode().strip()
        
        with stack_lock:
            imu_data_stack.append(imu_data)
            if len(imu_data_stack) > 10:
                imu_data_stack.pop(0)

# Function to transform the current reading to the desired zero-based orientation
def transform_to_desired_zero(current_real_orientation):
    delta_orientation = initialRealOrientation.inv() * current_real_orientation
    return desired_zero_orientation * delta_orientation

# Function to compute the best angle from the given joint angles
def compute_best_angle(joint_angles, prev_q1, prev_q3, prev_q5):
    next_angle = []
    if joint_angles == "no solution":
        return "no solution"
    elif len(joint_angles) > 1:
        work0 = abs(prev_q1 - joint_angles[0][0]) + abs(prev_q3 - joint_angles[0][1]) + abs(prev_q5 - joint_angles[0][2])
        work1 = abs(prev_q1 - joint_angles[1][0]) + abs(prev_q3 - joint_angles[1][1]) + abs(prev_q5 - joint_angles[1][2])
        if work0 < work1:
            next_angle = joint_angles[0]
        elif work1 < work0:
            next_angle = joint_angles[1]
        else:
            if abs(prev_q1 - joint_angles[0][0]) <= abs(prev_q1 - joint_angles[1][0]):
                next_angle = joint_angles[0]
    elif len(joint_angles) == 1:
        next_angle = joint_angles[0]

    return next_angle

# Function to read IMU data, perform calculations, and send messages to the server
def calculation_thread():
    global imu_data_stack
    global stack_lock
    global timeelapsed
    global bestangles
    global prevq1
    global prevq3
    global prevq5

    serial_number = 0
    serial_number2 = 1

    while True:
        with stack_lock:
            if len(imu_data_stack) > 0:
                imu_data = imu_data_stack.pop()
                current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(current_time + " - " + str(serial_number) + " - " + imu_data)
                serial_number += 1
                qvalues = imu_data.split('$')

                if len(qvalues) > 4:
                    qw = float(qvalues[1])
                    qx = float(qvalues[2])
                    qy = float(qvalues[3])
                    qz = float(qvalues[4])

                    if timeelapsed == 0:
                        # To store the first value of IMU
                        initialRealOrientation = R.from_quat(np.array([qw, qx, qy, qz]))
                        timeelapsed = round(time.time() * 1000)

                    torus = PlebCEngine.PlebcEngine()
                    quat = np.array([qw, qx, qy, qz])

                    relative_orientation = transform_to_desired_zero(R.from_quat(quat))
                    joint_angles = torus.JointAnglesFromquat(quat, prevq1, prevq3, prevq5)

                    bestangles = compute_best_angle(joint_angles, prevq1, prevq3, prevq5)

                    if bestangles != "no solution":
                        prevq1, prevq3, prevq5 = bestangles

                        message = f"{bestangles[0]}${bestangles[1]}${bestangles[2]}"
                        client.send_message(message)
                        # print(message)


                        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]            
                        print(current_time + " - " + str(serial_number2) + " - " + message)
                        serial_number2 += 1

                timeelapsed = round(time.time() * 1000)
                imu_data_stack.clear()

        time.sleep(0.1)


# Start the thread for reading IMU data
imu_thread = threading.Thread(target=imu_data_thread)
imu_thread.start()

# Start the thread for performing calculations
calculation_thread = threading.Thread(target=calculation_thread)
calculation_thread.start()

# The rest of your code...
# Add any additional functionality or main thread logic here.
# Ensure proper synchronization if necessary.
