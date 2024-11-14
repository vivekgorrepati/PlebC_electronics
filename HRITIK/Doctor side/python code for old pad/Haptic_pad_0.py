import time
import serial
import re

# Define the serial ports and baud rates for the IMU and controller
haptic_pad_port = 'COM22'  # Replace with your IMU's serial port
haptic_pad_rate = 115200    # Replace with your IMU's baud rate

# Create the serial connection for the IMU
haptic_pad_ser = serial.Serial(haptic_pad_port, haptic_pad_rate)

Controller_serial_port = 'COM3'  # Replace with your local sysytem controller serial port
Controller_baud_rate = 115200     # Replace with your local syatem controller's baud rate

# Create the serial connection for the controller
controller_ser = serial.Serial(Controller_serial_port, Controller_baud_rate)

# Variables to store the extracted values
gantry_values = ""
force_value = None
quaternion_values = ""
count = 0
while True:
    # Read data from the Haptic_pad's serial port
    
    received_string = haptic_pad_ser.readline().decode().strip()

    #---------Extract gantry values form string then saving in gantry_values---------
    gantry_match = re.search(r'(?<=Gantry_values : ).*', received_string)
    if gantry_match:
        gantry_values = gantry_match.group().strip()

    # -------------Here string is saving as it is in the gantry_values
    # gantry_match = re.search(r'Gantry_values : .*', received_string)
    # if gantry_match:
    #     gantry_values = gantry_match.group(0)
    
        
    # Printing the extracted values for this iteration
    print(gantry_values)
    gantry_values = gantry_values + "\n"
    if count == 1:
        controller_ser.write(gantry_values.encode()) 
        # controller_ser.flush()
        count = 0    
    count = count + 1
