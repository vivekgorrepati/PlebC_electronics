import PlebCEngine
import SocketCode
import math as m
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import serial

from datetime import datetime

serial_number = 1
serial_number2 = 1

websocket_url = "wss://torus-server-thxfxagavq-uc.a.run.app/"  # Replace with your WebSocket server URL
client = SocketCode.WebSocketClient(websocket_url)
client.start()
# Define the serial port and baud rate
serial_port = 'COM3'  # Replace with your Arduino's serial port
baud_rate = 115200  # Replace with your Arduino's baud rate


# Create the serial connection
imu_ser = serial.Serial(serial_port, baud_rate)

i=1

prevq1=0
prevq3=0
prevq5=0

# Capture the real initial orientation from the IMU
initialRealOrientation = R.from_quat([1,0,0,0])    # Actual IMU reading
desired_zero_orientation = R.from_quat([0,0,1,0])  # Desired 'zero' orientation

def transform_to_desired_zero(current_real_orientation):
    # Calculate the relative orientation
    delta_orientation = initialRealOrientation.inv() * current_real_orientation
    # Apply the delta to the desired "zero" orientation
    return desired_zero_orientation * delta_orientation

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
count = 0

while True:
    # Read a line of data from the serial connection
    imu_data = imu_ser.readline().decode().strip()

    # current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    # print(current_time + "--" + str(serial_number) + "-" + imu_data)
    # serial_number += 1

  
    # Split the line into individual values
    values = imu_data.split('$')
    
    # Process the quaternion values
    if len(values) > 4:
        q=m.radians(-50)
        start_time = time.time()
        qw=float(values[1])
        qx=float(values[2])
        qy=float(values[3])
        qz=float(values[4])

        # To store the first value of IMU
        if  count<1:
            initialRealOrientation = R.from_quat(np.array([qw,qx,qy,qz]))
            count=count+1
                
        lambi = PlebCEngine.FK()
        torus = PlebCEngine.PlebcEngine()
        
        quat  = np.array([qw,qx,qy,qz])
        
        # Transform the current reading to the desired zero-based orientation
        relative_orientation = transform_to_desired_zero(R.from_quat(quat))
        #print("relative orientation : ", relative_orientation.as_quat())
                    
        jointAngles= torus.JointAnglesFromquat(quat,prevq1,prevq3,prevq5)
        
        bestangles = computeBestAngle(jointAngles,prevq1,prevq3,prevq5)
               
        if bestangles!="no solution":
            prevq1 = bestangles[0]
            prevq3 = bestangles[1]
            prevq5 = bestangles[2]

            # print("sending message to server at :",time.time()*1000)
            message = str(bestangles[0])+"$"+str(bestangles[1])+"$"+str(bestangles[2])
            # client.send_message(message)
            #print(message)
            client.send_message(str(serial_number) + "-" + message)            
            current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]            
            print(current_time + " - " + str(serial_number2) + " - " + message)
            serial_number2 += 1
      
        timeelapsed = round(time.time()*1000)
        