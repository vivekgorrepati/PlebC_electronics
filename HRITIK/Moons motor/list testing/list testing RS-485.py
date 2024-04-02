import time
import minimalmodbus
from datetime import datetime
import random

# Create a Modbus RTU motor1
motor1 = minimalmodbus.Instrument('COM15', 1)  # Replace with your device's serial port and slave address
# Set the communication parameters for motor1 (baudrate, parity, etc.)
motor1.serial.baudrate = 9600
motor1.serial.bytesize = 8
motor1.serial.parity = minimalmodbus.serial.PARITY_NONE
motor1.serial.stopbits = 1
# motor1.serial.timeout = 0.1 
# motor1.serial.xonxoff = False
# motor1.serial.rtscts = False
# motor1.serial.dsrdtr = False
# motor1.serial.writeTimeout = 0

current_steps = 0

# List of 500 angles between -360 and 360
# angles_list = [i for i in range(-360, 361)]
angles_list = [270,-270,30,270,-270,180,270,-270,40,270,-270,270,-270,270,-270,0]

# Generate a list of 500 random integers between -360 and 360
# angles_list = [random.randint(-3600, 3600) for _ in range(1000)]  # Generate 499 random angles

# Add 0 as the last value in the list
# angles_list.append(0)

for serial_number, angle in enumerate(angles_list, start=1):
    try:
        print(serial_number, angle,  " time:", datetime.now().strftime("%H:%M:%S.%f")[:-3])
        target_steps = int((20000/360) * angle)  # to move forward write positive steps, for backward write negative steps
        steps = target_steps - current_steps
        values_to_write = [(steps >> 16) & 0xFFFF, steps & 0xFFFF]  # Split the 32-bit integer into two 16-bit values

        print( " Before writing time:", datetime.now().strftime("%H:%M:%S.%f")[:-3])
        motor1.write_registers(30, values_to_write)  # Write the values to the registers
        motor1.write_register(124, 102, functioncode=6)  # Send command to control register
        print( " After writing time:", datetime.now().strftime("%H:%M:%S.%f")[:-3])
        current_steps = current_steps + steps  # Updating steps
        
        time.sleep(0.300)
        print( "Before reading time:", datetime.now().strftime("%H:%M:%S.%f")[:-3])
        registers_values = motor1.read_registers(4, 2)  # Read the values from two consecutive registers (registers 5 and 6) Encoder value
        print( "After reading time:", datetime.now().strftime("%H:%M:%S.%f")[:-3])
        msbs = registers_values[0]  # Extract the Most Significant Bits (MSBs) from the long integer
        encoder_value = msbs << 16 | registers_values[1]  # Combine the MSBs to form the complete 32-bit integer
        print("Encoder value :", encoder_value, "Encoder value time:", datetime.now().strftime("%H:%M:%S.%f")[:-3])
        print("")
        
        
        
    except Exception as e:
        print(f"Error writing to holding registers: {str(e)}")

    finally:
        # Close the communication port (assuming you want to close it after each iteration)
        motor1.serial.close()