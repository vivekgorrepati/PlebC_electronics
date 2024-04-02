import time
import minimalmodbus
import threading

# Create a Modbus RTU motor1
motor1 = minimalmodbus.Instrument('COM15', 1)  # Replace with your device's serial port and slave address
# Set the communication parameters for motor1 (baudrate, parity, etc.)
motor1.serial.baudrate = 9600
motor1.serial.bytesize = 8
motor1.serial.parity = minimalmodbus.serial.PARITY_NONE
motor1.serial.stopbits = 1
motor1.serial.timeout = 0.1 
motor1.serial.xonxoff = False
motor1.serial.rtscts = False
motor1.serial.dsrdtr = False
motor1.serial.writeTimeout = 0

# Create a Modbus RTU motor2
motor2 = minimalmodbus.Instrument('COM15', 2)  # Replace with your device's serial port and slave address
# Set the communication parameters for motor2 (baudrate, parity, etc.)
motor2.serial.baudrate = 9600
motor2.serial.bytesize = 8
motor2.serial.parity = minimalmodbus.serial.PARITY_NONE
motor2.serial.stopbits = 1
motor2.serial.timeout = 0.1 
motor2.serial.xonxoff = False
motor2.serial.rtscts = False
motor2.serial.dsrdtr = False
motor2.serial.writeTimeout = 0

# Create a Modbus RTU motor3
motor3 = minimalmodbus.Instrument('COM15', 3)  # Replace with your device's serial port and slave address
# Set the communication parameters for motor3 (baudrate, parity, etc.)
motor3.serial.baudrate = 9600
motor3.serial.bytesize = 8
motor3.serial.parity = minimalmodbus.serial.PARITY_NONE
motor3.serial.stopbits = 1
motor3.serial.timeout = 0.1 
motor3.serial.xonxoff = False
motor3.serial.rtscts = False
motor3.serial.dsrdtr = False
motor3.serial.writeTimeout = 0

# Initialize current steps for each motor
current_steps_1 = 0
current_steps_2 = 0
current_steps_3 = 0

def motor1_operation(current_steps_1, angle_1):
    try:
        target_steps_1 = int((20000/360) * angle_1)
        steps_1 = target_steps_1 - current_steps_1
        values_to_write_1 = [(steps_1 >> 16) & 0xFFFF, steps_1 & 0xFFFF]
        motor1.write_registers(30, values_to_write_1)
        motor1.write_register(124, 102, functioncode=6)
        print("current steps :", current_steps_1)
        current_steps_1 = current_steps_1 + steps_1
        time.sleep(0.8)
        registers_values_1 = motor1.read_registers(4, 2)
        msbs_1 = registers_values_1[0]
        encoder_value_1 = msbs_1 << 16 | registers_values_1[1]
        print(f"Encoder value: {encoder_value_1} for motor {motor1.address}")
        time.sleep(1)

    except Exception as e:
        print(f"Error with motor {motor1.address}: {str(e)}")
        
def motor2_operation( current_steps_2, angle_2):
    try:
        target_steps_2 = int((20000/360) * angle_2)
        steps_2 = target_steps_2 - current_steps_2
        values_to_write_2 = [(steps_2 >> 16) & 0xFFFF, steps_2 & 0xFFFF]
        motor2.write_registers(30, values_to_write_2)
        motor2.write_register(124, 102, functioncode=6)
        print("current steps :", current_steps_2)
        current_steps_2 = current_steps_2 + steps_2
        time.sleep(0.8)
        registers_values_2 = motor2.read_registers(4, 2)
        msbs_2 = registers_values_2[0]
        encoder_value_2 = msbs_2 << 16 | registers_values_2[1]
        print(f"Encoder value: {encoder_value_2} for motor {motor2.address}")

    except Exception as e:
        print(f"Error with motor {motor2.address}: {str(e)}")
        
def motor3_operation(current_steps_3, angle_3):
    try:
        target_steps_3 = int((20000/360) * angle_3)
        steps_3 = target_steps_3 - current_steps_3
        values_to_write_3 = [(steps_3 >> 16) & 0xFFFF, steps_3 & 0xFFFF]
        motor3.write_registers(30, values_to_write_3)
        motor3.write_register(124, 102, functioncode=6)
        print("current steps :", current_steps_3)
        current_steps_3 = current_steps_3 + steps_3
        time.sleep(0.8)
        registers_values_3 = motor3.read_registers(4, 2)
        msbs_3 = registers_values_3[0]
        encoder_value_3 = msbs_3 << 16 | registers_values_3[1]
        print(f"Encoder value: {encoder_value_3} for motor {motor3.address}")

    except Exception as e:
        print(f"Error with motor {motor3.address}: {str(e)}")

# # Create motor instances
# motor1 = minimalmodbus.Instrument('COM15', 1)
# motor2 = minimalmodbus.Instrument('COM15', 2)
# motor3 = minimalmodbus.Instrument('COM15', 3)



while True:
    try:
        # Read input string containing angles
        input_angles = input("Enter angles : ")
        
        # Split the string using the $ delimiter
        angle_list = input_angles.split('$')
        
        # Assign angles to individual variables
        angle_1 = float(angle_list[0])
        angle_2 = float(angle_list[1])
        angle_3 = float(angle_list[2])

        # Create threads for motor operations
        thread1 = threading.Thread(target=motor1_operation, args=(current_steps_1, angle_1))
        thread2 = threading.Thread(target=motor2_operation, args=(current_steps_2, angle_2))
        thread3 = threading.Thread(target=motor3_operation, args=(current_steps_3, angle_3))

        # Start the threads
        thread1.start()
        thread2.start()
        thread3.start()
        

        # Wait for all threads to finish
        thread1.join()
        thread2.join()
        thread3.join()

    except Exception as e:
        print(f"Error: {str(e)}")
