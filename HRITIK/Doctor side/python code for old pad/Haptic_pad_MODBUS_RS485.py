import time
import serial
import re
import minimalmodbus
from datetime import datetime

# Configure the serial settings (adjust these according to your setup)
slave_id = 1
PORT = 'COM15'  
BAUDRATE = 115200
TIMEOUT = 0.4

#CONFIGURATION OF INSTRUMENT 1
# Create an instrument instance
instrument1 = minimalmodbus.Instrument(PORT, slave_id)  # Port name, slave address
instrument1.serial.baudrate = BAUDRATE
instrument1.serial.timeout = TIMEOUT
# Set other serial settings if necessary
instrument1.serial.parity = serial.PARITY_NONE
instrument1.serial.stopbits = 1
instrument1.serial.bytesize = 8

#CONFIGURATION OF INSTRUMENT 2
# Create an instrument instance
instrument2 = minimalmodbus.Instrument(PORT, slave_id)  # Port name, slave address
instrument2.serial.baudrate = BAUDRATE
instrument2.serial.timeout = TIMEOUT
# Set other serial settings if necessary
instrument2.serial.parity = serial.PARITY_NONE
instrument2.serial.stopbits = 1
instrument2.serial.bytesize = 8

#CONFIGURATION OF INSTRUMENT 3
# Create an instrument instance
instrument3 = minimalmodbus.Instrument(PORT, slave_id)  # Port name, slave address
instrument3.serial.baudrate = BAUDRATE
instrument3.serial.timeout = TIMEOUT
# Set othe3 serial settings if necessary
instrument3.serial.parity = serial.PARITY_NONE
instrument3.serial.stopbits = 1
instrument3.serial.bytesize = 8


# Define the serial ports and baud rates for the IMU and controller
haptic_pad_port = 'COM22'  # Replace with your IMU's serial port
haptic_pad_rate = 115200    # Replace with your IMU's baud rate

# Create the serial connection for the IMU
haptic_pad_ser = serial.Serial(haptic_pad_port, haptic_pad_rate)

# Variables to store the extracted values
gantry_values = ""
force_value = None
quaternion_values = ""
x = None
y = None
z = None

prev_x = "0"
prev_y = "0"
prev_z = "0"

while True:
    # Read data from the Haptic_pad's serial port
    received_string = haptic_pad_ser.readline().decode().strip()

    #---------Extract gantry values from string then saving in gantry_values---------
    gantry_match = re.search(r'(?<=Gantry_values : ).*', received_string)
    if gantry_match:
        gantry_values = gantry_match.group().strip()
        
        # Split the string by '$' and extract positions
        gantry_list = gantry_values.split('$')
        
        # Ensure the string has the expected number of parts (6 parts)
        if len(gantry_list) == 6:
            x = gantry_list[2]  # -x1
            y = gantry_list[3]  # y1
            z = gantry_list[4]  # x2

            # Convert x, y, z to strings (if not already)
            x = str(x)
            y = str(y)
            z = str(z)
            
            # Check if the current x is different from the previous one
            if x != prev_x:
                if x == "0":
                    instrument1.write_register(4, 0,functioncode=6)
                elif x == "x1":
                    instrument1.write_register(4, 1,functioncode=6)
                elif x == "-x1":
                    instrument1.write_register(4, 2,functioncode=6)

            # Check if the current y is different from the previous one
            if y != prev_y:
                if y == "0":
                    instrument1.write_register(4, 0,functioncode=6)
                elif y == "y1":
                    instrument1.write_register(4, 1,functioncode=6)
                elif y == "-y1":
                    instrument1.write_register(4, 2,functioncode=6)

            # Check if the current z is different from the previous one
            if z != prev_z:
                if z == "0":
                    instrument1.write_register(4, 0,functioncode=6)
                elif z == "x2":
                    instrument1.write_register(4, 1,functioncode=6)
                elif z == "-x2":
                    instrument1.write_register(4, 2,functioncode=6)

            # Update prev_x to the current value of x
            prev_x = x
            prev_y = y
            prev_z = z

    time.sleep(0.1)  # Adjust sleep duration if necessary


   
    
        
    