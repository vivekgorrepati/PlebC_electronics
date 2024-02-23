import PlebCEngine
import math as m
import numpy as np
from scipy.spatial.transform import Rotation as R
import re
import time
import serial

# Define the serial ports and baud rates for the IMU and controller
haptic_pad_port = 'COM11'  # Replace with your IMU's serial port
haptic_pad_rate = 115200    # Replace with your IMU's baud rate

Controller_serial_port = 'COM3'  # Replace with your local sysytem controller serial port
Controller_baud_rate = 9600     # Replace with your local syatem controller's baud rate

# Create the serial connection for the IMU
haptic_pad_ser = serial.Serial(haptic_pad_port, haptic_pad_rate)

# Create the serial connection for the controller
controller_ser = serial.Serial(Controller_serial_port, Controller_baud_rate)

# Variables to store the extracted values
gantry_values = ""
force_value = None
quaternion_values = ""

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
    # Read data from the Haptic_pad's serial port
    
    received_string = haptic_pad_ser.readline().decode().strip()
    print(received_string)

    #---------Extract gantry values form string then saving in gantry_values---------
    # gantry_match = re.search(r'(?<=Gantry_values : ).*', received_string)
    # if gantry_match:
    #     gantry_values = gantry_match.group().strip()

    # -------------Here string is saving as it is in the gantry_values
    gantry_match = re.search(r'Gantry_values : .*', received_string)
    if gantry_match:
        gantry_values = gantry_match.group(0)
    # --------------------------------------------------------------------------------

    # -------------Extract force value form string then saving in force_values
    # force_match = re.search(r'(?<=Force : )\d+', received_string)
    # if force_match:
    #     force_value = force_match.group().strip()
    
    # Here string is saving as it is in the force_values which have force data
    force_match = re.search(r'Force : .*', received_string)
    if force_match:
        force_value = force_match.group(0)
    # ----------------------------------------------------------------------------------
    # ----------------Extract quaternion values
    quat_match = re.search(r'(?<=quat_value:).*', received_string)
    if quat_match:
        quaternion_values = quat_match.group().strip()
    # -----------------------------------------------------------------------------
        
    # Printing the extracted values for this iteration
    # print("Gantry Values :",gantry_values)
    # print("Force :",force_value)
    # print("Quaternion values :",quaternion_values)      
    
    # Send Gantry values to Patient Side
    controller_ser.write(gantry_values.encode())
    print(gantry_values)

    # Send force values to Patient Side
    # controller_ser.write(force_value.encode())
    print(force_value)

    # Split the line into individual values
    values = quaternion_values.split('$')
    # print(time.time()*1000-timeelapsed)
    # print(values)
    timeelapsed = time.time()*2000
    
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
        # print("relative orientation : ", relative_orientation.as_quat())
        
        jointAngles= torus.JointAnglesFromquat(relative_orientation.as_quat(),prevq1,prevq3,prevq5)
        
        bestangles = computeBestAngle(jointAngles,prevq1,prevq3,prevq5)
        # print(time.time()*1000-timeelapsed,": computed best angle \n",bestangles)
       
        if bestangles!="no solution":
            prevq1 = bestangles[0]
            prevq3 = bestangles[1]
            prevq5 = bestangles[2]
            # Prepare the data to send to the controller
            controller_data = f"{bestangles[0]} ${bestangles[1]} ${bestangles[2]}\n"

            # Send Quaternion values to the controller
            best_angles = "localSystem_value : " + controller_data
            controller_ser.write(best_angles.encode())            
            print(best_angles)                    

        timeelapsed = round(time.time()*1000)
        
    