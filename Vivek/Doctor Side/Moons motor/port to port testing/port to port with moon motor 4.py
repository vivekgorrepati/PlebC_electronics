import PlebCEngine
import math as m
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import serial

# Define the serial ports and baud rates for the IMU and controller
imu_serial_port = 'COM3'  # Replace with your IMU's serial port
imu_baud_rate = 115200    # Replace with your IMU's baud rate

# Create the serial connection for the IMU
imu_ser = serial.Serial(imu_serial_port, imu_baud_rate)

# Initialization parameters for the motor serial connection
def motor_init():
    ser = serial.Serial()
    ser.port = "COM15"
    ser.baudrate = 9600
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = .1
    ser.xonxoff = False
    ser.rtscts = False
    ser.dsrdtr = False
    ser.writeTimeout = 0
    return ser

# When we send a serial command, the program will check and print
# the response given by the drive.
def send(ser, command):
    ser.write((command+'\r').encode())
    # response = ser.readline().decode().strip()
    raw_data = ser.readline().strip()
    response = bytes.fromhex(raw_data.decode('utf-8'))
    print(response)
    if len(response) > 0:
        ser.flushInput()
    return response  # Return the response

def motor_setup(ser):
    """Setup initial motor parameters, also resets alarm"""
    send(ser, 'EG20000')  # Sets microstepping to 20,000 steps per revolution
    send(ser, 'IFD')       # Sets the format of drive responses to decimal
    send(ser, 'SP0')       # Sets the starting position at 0
    send(ser, 'AR')        # Alarm reset
    send(ser, 'AC10')      # Acceleration
    send(ser, 'DE15')      # Deceleration
    send(ser, 'VE10')      # Velocity
    send(ser, 'ME')        # Enable Motor

current_position = 0

# Capture the real initial orientation from the IMU
initialRealOrientation = R.from_quat([1, 0, 0, 0])  # Actual IMU reading
desired_zero_orientation = R.from_quat([0, 0, 1, 0])  # Desired 'zero' orientation

def transform_to_desired_zero(current_real_orientation):
    # Calculate the relative orientation
    delta_orientation = initialRealOrientation.inv() * current_real_orientation
    # Apply the delta to the desired "zero" orientation
    return desired_zero_orientation * delta_orientation

prevq1 = 0
prevq3 = 0
prevq5 = 0
current_position = 0  # Variable to store the current position

def computeBestAngle(jointAngles, prevq1, prevq3, prevq5):
    NextAngle = []
    if jointAngles == "no solution":
        return "no solution"
    elif len(jointAngles) > 1:
        work0 = abs(prevq1 - jointAngles[0][0]) + abs(prevq3 - jointAngles[0][1]) + abs(prevq5 - jointAngles[0][2])
        work1 = abs(prevq1 - jointAngles[1][0]) + abs(prevq3 - jointAngles[1][1]) + abs(prevq5 - jointAngles[1][2])
        if work0 < work1:
            NextAngle = jointAngles[0]
        elif work1 < work0:
            NextAngle = jointAngles[1]
        else:
            if abs(prevq1 - jointAngles[0][0]) <= abs(prevq1 - jointAngles[1][0]):
                NextAngle = jointAngles[0]
    elif len(jointAngles) == 1:
        NextAngle = jointAngles[0]

    return NextAngle

timeelapsed = 0
bestangles = [0, 0, 0]  # Initialize with a default value
count = 0

def move(ser, target_position):
    global current_position
    steps = target_position - current_position
    send(ser, "FL" + str(steps))  
    print("FL" + str(steps))
    current_position = current_position + steps

    # This section demonstrates the drive's ability to poll immediate position
    # and check status to see if the move is done.
    # time.sleep(.5)
    send(ser, 'IP')  # IP is immediate position
    time.sleep(1.2)
    
    send(ser, 'IP')  # Read IP response
    # ip_response = ser.readline().strip().decode('utf-8')
    # print(f"Immediate Position Response: {ip_response}")
    time.sleep(1)
    send(ser, 'RS')  # Request the status of the drive.
    send(ser, '98')  # Request the status of the drive.

   

    if ser.isOpen():
        try:
            ser.flushInput()
            ser.flushOutput()
            motor_setup(ser)  # Complete motor setup and enable motor

        except Exception as e1:
            print("Error Communicating...: " + str(e1))
    else:
        print("Cannot open serial port")

# Open the serial port before entering the loop
ser = motor_init()
ser.open()
time.sleep(1)

while True:
    try:
        # Read data from the IMU's serial port
        imu_data = imu_ser.readline().decode().strip()
        # Split the line into individual values
        values = imu_data.split('$')

        # Process the quaternion values
        if len(values) > 4:
            qw = float(values[1])
            qx = float(values[2])
            qy = float(values[3])
            qz = float(values[4])

            # To store the first value of IMU
            if count < 1:
                initialRealOrientation = R.from_quat(np.array([qw, qx, qy, qz]))
                count += 1

            lambi = PlebCEngine.FK()
            torus = PlebCEngine.PlebcEngine()

            quat = np.array([qw, qx, qy, qz])

            # Transform the current reading to the desired zero-based orientation
            relative_orientation = transform_to_desired_zero(R.from_quat(quat))

            jointAngles = torus.JointAnglesFromquat(relative_orientation.as_quat(), prevq1, prevq3, prevq5)

            bestangles = computeBestAngle(jointAngles, prevq1, prevq3, prevq5)

            if bestangles != "no solution":
                prevq1 = bestangles[0]
                prevq3 = bestangles[1]
                prevq5 = bestangles[2]

                # Update the current position based on the movement
                target_position = int(bestangles[0] * (200000 / 360))
                print(bestangles[0])

                move(ser, target_position)

    except Exception as e:
        print(f"Error: {str(e)}")

# Close the serial port when done
# ser.close()
