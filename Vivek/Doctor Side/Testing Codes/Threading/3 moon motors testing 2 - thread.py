import time
import minimalmodbus
import threading

def motor_operation(motor, current_steps, angle):
    try:
        target_steps = int((20000/360) * angle)
        steps = target_steps - current_steps
        values_to_write = [(steps >> 16) & 0xFFFF, steps & 0xFFFF]
        motor.write_registers(30, values_to_write)
        motor.write_register(124, 102, functioncode=6)
        current_steps = current_steps + steps
        time.sleep(0.4)
        registers_values = motor.read_registers(4, 2)
        msbs = registers_values[0]
        encoder_value = msbs << 16 | registers_values[1]
        print(f"Encoder value: {encoder_value} for motor {motor.address}")

    except Exception as e:
        print(f"Error with motor {motor.address}: {str(e)}")

# Create motor instances
motor1 = minimalmodbus.Instrument('COM15', 1)
motor2 = minimalmodbus.Instrument('COM15', 2)
motor3 = minimalmodbus.Instrument('COM15', 3)

# Set communication parameters for motors
for motor in [motor1, motor2, motor3]:
    motor.serial.baudrate = 9600
    motor.serial.bytesize = 8
    motor.serial.parity = minimalmodbus.serial.PARITY_NONE
    motor.serial.stopbits = 1
    motor.serial.timeout = 0.1 
    motor.serial.xonxoff = False
    motor.serial.rtscts = False
    motor.serial.dsrdtr = False
    motor.serial.writeTimeout = 0

# Initialize current steps for each motor
current_steps_1 = 0
current_steps_2 = 0
current_steps_3 = 0

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
        thread1 = threading.Thread(target=motor_operation, args=(motor1, current_steps_1, angle_1))
        thread2 = threading.Thread(target=motor_operation, args=(motor2, current_steps_2, angle_2))
        thread3 = threading.Thread(target=motor_operation, args=(motor3, current_steps_3, angle_3))

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