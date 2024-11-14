import serial
import minimalmodbus
from datetime import datetime
import time
import re
# Define the serial ports and baud rates for the IMU and controller
joystick_port = 'COM16'  # Replace with your IMU's serial port
joystick_baud_rate = 9600    # Replace with your IMU's baud rate

# Create the serial connection for the Joystick
joystick_ser = serial.Serial(joystick_port, joystick_baud_rate)

# Configure the serial settings (adjust these according to your setup)
PORT = 'COM15'  # or '/dev/ttyUSB0' for Linux
BAUDRATE = 115200
TIMEOUT = 1

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

prev_holding_registers = 0
prev_X_values = 0
prev_Y_values = 0
prev_Z_values = 0
prev_RPM_values=0

X_values = 0
Y_values = 0
Z_values = 0
RPM_values=0

while True :
    # Read data from the serial port
    joystick_data = joystick_ser.readline().decode().strip()

    X_match = re.search(r'(?<=X :).*', joystick_data)
    if X_match:
        X_values = int(X_match.group().strip())
        print("X : ",X_values)

             
    
    # Y_match = re.search(r'(?<=Y :).*', joystick_data)
    # if Y_match:
    #     Y_values = int(Y_match.group().strip())
    #     print("Y : ",Y_values)
    

    # Z_match = re.search(r'(?<=Z :).*', joystick_data)
    # if Z_match:
    #     Z_values = int(Z_match.group().strip())
    #     print("Z : ",Z_values)

    # RPM_match = re.search(r'(?<=V :).*', joystick_data)
    # if RPM_match:
    #     RPM_values = int(RPM_match.group().strip())
    #     if 0 <= RPM_values <= 100 :
    #         instrument1.write_register(0, RPM_values,functioncode=6)
    #         print("Velocity : ",RPM_values)

    if X_values >= 0  & X_values <= 900 :
        if X_values != prev_X_values :
            instrument1.write_register(4, X_values,functioncode=6)
            # holding_registers = instrument1.read_registers(4, 1, functioncode=3)
            # print("Holding Registers 4 of Slave 1:", holding_registers)  
            
    # current_distance = instrument1.read_registers(1, 1, functioncode=4)
    # print("current distance :",current_distance)

    # current_distance = instrument1.read_registers(0, 1, functioncode=4)
    # print("current Encoder :",current_distance)

    # stop=instrument1.write_register(5,3,functioncode=4)
    # instrument1.read_register(4, stop,functioncode=6)
    # print("motor stopped")


    # if Y_values >= 0  & Y_values <= 900 :
    # 	if Y_values != prev_Y_values :
    #         instrument2.write_register(4, Y_values,functioncode=6)
    #         holding_registers = instrument2.read_registers(4, 2, functioncode=3)
    #         print("Holding Registers 4 of Slave 2:", holding_registers)

    # if Z_values >= 0  & Z_values <= 900 :
    #     if Z_values != prev_Z_values :
    #         instrument3.write_register(4, Z_values,functioncode=6)
    #         holding_registers = instrument3.read_registers(4, 3, functioncode=3)
    #         print("Holding Registers 4 of Slave 3:", holding_registers)

    prev_X_values = X_values

# while True :
   
    # joystick_data = joystick_ser.readline().decode().strip()
    # print(joystick_data)    
# before = datetime.now().strftime("%H:%M:%S.%f")[:-3]
# holding_registers1 = instrument1.read_registers(4, 3, functioncode=3)
# holding_registers2 = instrument2.read_registers(4, 3, functioncode=3)
# holding_registers3 = instrument3.read_registers(4, 3, functioncode=3)
# after = datetime.now().strftime("%H:%M:%S.%f")[:-3]


# print("Before reading ", before)
# print("Holding Registers 4 of Slave 1:", holding_registers1)
# # time.sleep(0.200)

# print("Holding Registers 4 of Slave 2:", holding_registers2)
# # time.sleep(0.200)

# print("Holding Registers 4 of Slave 3:", holding_registers3)
# # time.sleep(0.200)

# print("After reading ", after)


# instrument1.write_register(0, 50,functioncode=6)
# time.sleep(0.200)
# instrument1.write_register(4, 100,functioncode=6)

# while True :
#     input_registers1 = instrument1.read_registers(1, 1, functioncode=4)
#     print(input_registers1)

#     if input_registers1[0] == 0 :
#         instrument1.write_register(4, 100,functioncode=6)
#     if input_registers1[0] == 100 :
#         instrument1.write_register(4, 0,functioncode=6)


# instrument1.write_register(0, 5,functioncode=6)
# while True :
# distancez= int(input("Enter Distance z : "))
# # distancex= int(input("Enter Distance x : "))
# instrument1.write_register(4, 500,functioncode=6)
# # instrument2.write_register(4, distancex,functioncode=6)



# instrument1.write_register(4,400,functioncode=6)
# time.sleep(2)
# instrument1.write_register(6, 1,functioncode=6)



# instrument1.write_register(0,2,functioncode=6)
# instrument1.write_register(4,40,functioncode=6)



