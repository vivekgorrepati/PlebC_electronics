import PlebCEngine
import math as m
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import serial

# Define the serial ports and baud rates for the IMU and controller
imu_serial_port = 'COM3'  # Replace with your IMU's serial port
imu_baud_rate = 115200    # Replace with your IMU's baud rate

# Controller_serial_port = 'COM14'  # Replace with your local sysytem controller serial port
# Controller_baud_rate = 9600     # Replace with your local syatem controller's baud rate

# Create the serial connection for the IMU
imu_ser = serial.Serial(imu_serial_port, imu_baud_rate)

# Create the serial connection for the controller
# controller_ser = serial.Serial(Controller_serial_port, Controller_baud_rate)

# Initialization parameters. Note the serial port and baud rate of your project
# may vary. Our default baud rate is 9600
def motor_init():
    ser=serial.Serial()
    ser.port = "COM6"
    ser.baudrate = 9600
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout= .1
    ser.xonxoff = False
    ser.rtscts = False
    ser.dsrdtr = False
    ser.writeTimeout = 0
    return ser

# When we send a serial command, the program will check and print
# the response given by the drive.
def send(ser,command):
        ser.write((command+'\r').encode())
        response = ser.read(15).decode()
        if len(response) > 0:
            print (response)
            ser.flushInput()

def motor_setup(ser):
        
        """Setup initial motor parameters, also resets alarm"""

        send(ser,'EG20000') # Sets microstepping to 20,000 steps per revolution
        send(ser,'IFD') # Sets the format of drive responses to decimal
        send(ser,'SP0') # Sets the starting position at 0
        send(ser,'AR') # Alarm reset
        send(ser,'AC10') # Acceleration 
        send(ser,'DE15') # Deceleration
        send(ser,'VE10') # Velocity
        send(ser,'ME')  # Enable Motor

# Capture the real initial orientation from the IMU
initialRealOrientation = R.from_quat([1,0,0,0])    # Actual IMU reading
desired_zero_orientation = R.from_quat([0,0,1,0])  # Desired 'zero' orientation

def transform_to_desired_zero(current_real_orientation):
    # Calculate the relative orientation
    delta_orientation = initialRealOrientation.inv() * current_real_orientation
    # Apply the delta to the desired "zero" orientation
    return desired_zero_orientation * delta_orientation

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
count = 0

while True:
    # Read data from the IMU's serial port
    imu_data = imu_ser.readline().decode().strip()
    print(imu_data)    
    
    # Split the line into individual values
    values = imu_data.split('$')
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
        print("relative orientation : ", relative_orientation.as_quat())
        
        jointAngles= torus.JointAnglesFromquat(relative_orientation.as_quat(),prevq1,prevq3,prevq5)
        
        bestangles = computeBestAngle(jointAngles,prevq1,prevq3,prevq5)
        # print(time.time()*1000-timeelapsed,": computed best angle \n",bestangles)
       
        if bestangles!="no solution":
            prevq1 = bestangles[0]
            prevq3 = bestangles[1]
            prevq5 = bestangles[2]

            # Prepare the data to send to the controller
            controller_data = f"{bestangles[0]} ${bestangles[1]} ${bestangles[2]}\n"

            # Send the data to the controller
        #     controller_ser.write(controller_data.encode())            
        #     print(controller_data)

        # timeelapsed = round(time.time()*1000)
            
            

    def move(ser):
        send(ser,'FL60000') # Moves 3 revs CW
        send(ser,'FL-120000') #Moves 6 revs CCW
        """
        This section demonstrates the drives ability to poll immediate position
        and check status to see if the move is done.
                """
        time.sleep(.5)
        send(ser, 'IP') # IP is immediate position
        time.sleep(1.2)
        send(ser,'IP')
        time.sleep(1)
        send(ser,'RS') # We end by requesting the status of the drive.

        try:
            ser = motor_init() # Initialise the serial port
            ser.open()         # open the serial port
            time.sleep(1)
        except Exception as e:
            print ('error opening serial port')
            exit()

        if ser.isOpen():
            try:
                ser.flushInput()
                ser.flushOutput()
                motor_setup(ser)  # Complete motor setup and enable motor
                move(ser)         # perform the move command
                
            except Exception as e1:
                print ("Error Communicating...: " + str(e1))
        else:
            print ("Cannot open serial port ")

                
            