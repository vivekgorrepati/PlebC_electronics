import time
import minimalmodbus
from datetime import datetime

# Create a Modbus RTU motor1
motor1 = minimalmodbus.Instrument('COM15', 1)  # Replace with your device's serial port and slave address
# Set the communication parameters for motor1 (baudrate, parity, etc.)
motor1.serial.baudrate = 115200
motor1.serial.bytesize = 8
motor1.serial.parity = minimalmodbus.serial.PARITY_NONE
motor1.serial.stopbits = 1
motor1.serial.timeout = 0.1 
motor1.serial.xonxoff = False
motor1.serial.rtscts = False
motor1.serial.dsrdtr = False
motor1.serial.writeTimeout = 0

current_steps = 0

while True:
    try:
        user_input = input("Enter angle or 's' to stop: ")
        
        if user_input.lower() == 's':
            # print( "Before stop time:", datetime.now().strftime("%H:%M:%S.%f")[:-3])
            motor1.write_register(124, 225)  # stop command            
            # print( "After stop time:", datetime.now().strftime("%H:%M:%S.%f")[:-3])
            continue  # Skip the rest of the loop and wait for the next input
        
        angle = int(user_input)
        target_steps = int((20000/360) * angle)  # to move forward write positive steps, for backward write negative steps
        steps = target_steps - current_steps        
        values_to_write = [(steps >> 16) & 0xFFFF, steps & 0xFFFF] # Split the 32-bit integer into two 16-bit values
        
        motor1.write_registers(30, values_to_write) # Write the values to the registers
        motor1.write_register(124, 102, functioncode=6) # send command to control register
        current_steps = current_steps + steps  # Updating steps
        
        time.sleep(0.04)             
        # registers_values = motor1.read_registers(4, 2) # Read the values from two consecutive registers (registers 5 and 6) Encoder value
        registers_values = motor1.read_register(5)
        # msbs = registers_values[0] # Extract the Most Significant Bits (MSBs) from the long integer
        # encoder_value = msbs << 16 | registers_values[1] # Combine the MSBs to form the complete 32-bit integer
        # print("Encoder value :", encoder_value )   
        print("Encoder value :", registers_values)
        
        
    except ValueError:
        print("Invalid input. Please enter an integer angle or 's' to stop.")
    
    except Exception as e:
        print(f"Error: {str(e)}")

# Close the communication port outside the loop
motor1.serial.close()
