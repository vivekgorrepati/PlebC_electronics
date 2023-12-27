import PlebCEngine
import SocketCode
import math as m
import numpy as np
from scipy.spatial.transform import Rotation
import time
import serial



websocket_url = "wss://torusserver-jhybo7-microtica.microtica.rocks"  # Replace with your WebSocket server URL
client = SocketCode.WebSocketClient(websocket_url)
client.start()
# Define the serial port and baud rate
serial_port = 'COM10'  # Replace with your Arduino's serial port
baud_rate = 115200  # Replace with your Arduino's baud rate


# Create the serial connection
ser = serial.Serial(serial_port, baud_rate)
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
while True:
    # Read a line of data from the serial connection
    line = ser.readline().decode().strip()

  
    # Split the line into individual values
    values = line.split('$')
    print(time.time()*1000-timeelapsed)
    print(values)
    timeelapsed = time.time()*1000
    # Process the quaternion values
    ''''
    if len(values) > 4:
        q=m.radians(-50);
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
        print(time.time()*1000-timeelapsed," : printing quat \n",quat)
        #timeelapsed = round(time.time()*1000)
        #wxyz = np.array([quat[3],quat[0],quat[1],quat[2]])
        #wxyz = np.array([0.0210,0.8502,-0.5201,-0.02001])
        #print("printing after swap to wxyz : ",wxyz)
    
        jointAngles= torus.JointAnglesFromquat(quat,prevq1,prevq3,prevq5)
        
        print(time.time()*1000-timeelapsed," joint angles\n",jointAngles)
        #timeelapsed = round(time.time()*1000)
        bestangles = computeBestAngle(jointAngles,prevq1,prevq3,prevq5)
        print(time.time()*1000-timeelapsed,": computed best angle \n",bestangles)
       
        if bestangles!="no solution":
            prevq1 = bestangles[0]
            prevq3 = bestangles[1]
            prevq5 = bestangles[2]
            print("sending message to server at :",time.time()*1000)
            client.send_message(str(bestangles[0])+"$"+str(bestangles[1])+"$"+str(bestangles[2]))
        #print("joint angles",jointAngles)       
      
        timeelapsed = round(time.time()*1000)
        
    
    '''

#start_time = time.time()

# Your program code goes here
'''
def quaternion_from_rotation_matrix(matrix):
    rotation = Rotation.from_matrix(matrix)
    quaternion = rotation.as_quat()
    return quaternion

lambi = PlebCEngine.FK()
torus = PlebCEngine.PlebcEngine()

q=m.radians(-50);
'''
#RotationMatrixFromJointAngles = lambi.fwdKin(q,m.radians(10),m.radians(180),m.radians(120),0.15,0.00,0.32);
#print(RotationMatrixFromJointAngles)
'''
r11 = RotationMatrixFromJointAngles[0,0]
r12 = RotationMatrixFromJointAngles[0,1]
r13 = RotationMatrixFromJointAngles[0,2]
r21 = RotationMatrixFromJointAngles[1,0]
r22 = RotationMatrixFromJointAngles[1,1]
r23 = RotationMatrixFromJointAngles[1,2]=
r31 = RotationMatrixFromJointAngles[2,0]
r32 = RotationMatrixFromJointAngles[2,1]
r33 = RotationMatrixFromJointAngles[2,2]

RotMatrix = np.array([r11, r12, r13,
                      r21, r22, r23,
                      r31, r32, r33]).reshape(3,3)
'''
#print(RotMatrix)
#quat = quaternion_from_rotation_matrix(RotMatrix)
#quat  = np.array([qw,qx,qy,qz])
#print("printing quat ",quat)




#wxyz = np.array([quat[3],quat[0],quat[1],quat[2]])
#wxyz = np.array([0.0210,0.8502,-0.5201,-0.02001])
#print("printing after swap to wxyz : ",wxyz)
#print(torus.JointAnglesFromquat(quat))
#end_time = time.time()
#execution_time = end_time - start_time
#print("Execution time: {execution_time} seconds")