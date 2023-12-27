import PlebCEngine
import math as m
import numpy as np
from scipy.spatial.transform import Rotation
import time
import serial

# Define the serial ports and baud rates for the IMU and controller
imu_serial_port = 'COM3'  # Replace with your IMU's serial port
imu_baud_rate = 115200    # Replace with your IMU's baud rate

Controller_serial_port = 'COM6'  # Replace with your local sysytem controller serial port
Controller_baud_rate = 9600     # Replace with your local syatem controller's baud rate



# Create the serial connection for the IMU
imu_ser = serial.Serial(imu_serial_port, imu_baud_rate)

# Create the serial connection for the controller
controller_ser = serial.Serial(Controller_serial_port, Controller_baud_rate)

i=1

prevq1=0
prevq3=0
prevq5=0


def computeBestAngle(jointAngles,prevq1,prevq3,prevq5):
    NextAngle = []
    if jointAngles == "no solution":
        return "no solution"
    elif len(jointAngles)>1:
        work0 = abs(prevq1-jointAngles[0][0])+abs(prevq3-jointAngles[0][1])+abs(prevq5-jointAngles[0][2])
        work1 = abs(prevq1-jointAngles[1][0])+abs(prevq3-jointAngles[1][1])+abs(prevq5-jointAngles[1][2])
        if work0<work1:
            NextAngle = jointAngles[0]
        elif work1<work0:
            NextAngle = jointAngles[1]
        else:
            if abs(prevq1-jointAngles[0][0])<=abs(prevq1-jointAngles[1][0]):
                NextAngle = jointAngles[0]
    elif len(jointAngles)==1:
        NextAngle = jointAngles[0]
    
    return NextAngle

timeelapsed = 0
bestangles = [0, 0, 0]  # Initialize with a default value
while True:
    # Read data from the IMU's serial port
    imu_data = imu_ser.readline().decode().strip()
    print(imu_data) 
    
    # Split the line into individual values
    values = imu_data.split('$')
    # print(time.time()*1000-timeelapsed)
    # print(values)
    timeelapsed = time.time()*1000
    
    # Process the quaternion values 
    if len(values) > 4:
        q=m.radians(-50)
        start_time = time.time()
        qw=float(values[1])
        qx=float(values[2])
        qy=float(values[3])
        qz=float(values[4])
        #q0, q1, q2, q3 = [float(v) for v in values]
        # Process the quaternion values as needed
        # Example: Print the quaternion values
        #print("Quaternion: ({qw}, {qx}, {qy}, {qz})")
        
        lambi = PlebCEngine.FK()
        torus = PlebCEngine.PlebcEngine()
        
        quat  = np.array([qw,qx,qy,qz])
        #print(type(qw))
        # print(time.time()*1000-timeelapsed," : printing quat \n",quat)
        #timeelapsed = round(time.time()*1000)
        #wxyz = np.array([quat[3],quat[0],quat[1],quat[2]])
        #wxyz = np.array([0.0210,0.8502,-0.5201,-0.02001])
        #print("printing after swap to wxyz : ",wxyz)
    
        jointAngles= torus.JointAnglesFromquat(quat,prevq1,prevq3,prevq5)
        
        # print(time.time()*1000-timeelapsed," joint angles\n",jointAngles)
        #timeelapsed = round(time.time()*1000)
        bestangles = computeBestAngle(jointAngles,prevq1,prevq3,prevq5)
        # print(time.time()*1000-timeelapsed,": computed best angle \n",bestangles)
       
        if bestangles!="no solution":
            prevq1 = bestangles[0]
            prevq3 = bestangles[1]
            prevq5 = bestangles[2]
            # Prepare the data to send to the controller
            controller_data = f"{bestangles[0]} ${bestangles[1]} ${bestangles[2]}\n"

            # Send the data to the controller
            controller_ser.write(controller_data.encode())
            print(controller_data)

        timeelapsed = round(time.time()*1000)
        