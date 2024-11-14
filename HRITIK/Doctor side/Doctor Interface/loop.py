import serial
import minimalmodbus
from datetime import datetime
import time
import re

# Configure the serial settings (adjust these according to your setup)
# slave_id = 1
PORT = 'COM15'  # or '/dev/ttyUSB0' for Linux
BAUDRATE = 115200
TIMEOUT = 0.4

#CONFIGURATION OF INSTRUMENT 1
# Create an instrument instance
instrument1 = minimalmodbus.Instrument(PORT, 1)  # Port name, slave address
instrument1.serial.baudrate = BAUDRATE
instrument1.serial.timeout = TIMEOUT

# Set other serial settings if necessary
instrument1.serial.parity = serial.PARITY_NONE
instrument1.serial.stopbits = 1
instrument1.serial.bytesize = 8

#CONFIGURATION OF INSTRUMENT 2
# Create an instrument instance
instrument2 = minimalmodbus.Instrument(PORT, 2)  # Port name, slave address
instrument2.serial.baudrate = BAUDRATE
instrument2.serial.timeout = TIMEOUT

# Set other serial settings if necessary
instrument2.serial.parity = serial.PARITY_NONE
instrument2.serial.stopbits = 1
instrument2.serial.bytesize = 8

#CONFIGURATION OF INSTRUMENT 3
# Create an instrument instance
instrument3 = minimalmodbus.Instrument(PORT, 3)  # Port name, slave address
instrument3.serial.baudrate = BAUDRATE
instrument3.serial.timeout = TIMEOUT
# Set other serial settings if necessary
instrument3.serial.parity = serial.PARITY_NONE
instrument3.serial.stopbits = 1
instrument3.serial.bytesize = 8

while True : 
    angle1 = int(input("Enter angle : "))
    angle2 = int(input("Enter angle : "))
    angle3 = int(input("Enter angle : "))
    instrument1.write_register(4, angle1,functioncode=6)
    instrument2.write_register(4, angle2*27,functioncode=6)
    instrument3.write_register(4, angle3,functioncode=6)