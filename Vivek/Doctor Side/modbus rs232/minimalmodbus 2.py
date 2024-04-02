import minimalmodbus
import time

# Create a Modbus RTU Driver_1
Driver_1 = minimalmodbus.Instrument('COM15', 1)  # Replace with your device's serial port and slave address

# Set the communication parameters (baudrate, parity, etc.)
Driver_1.serial.baudrate = 9600
Driver_1.serial.bytesize = 8  # EIGHTBITS
Driver_1.serial.parity = minimalmodbus.serial.PARITY_NONE
Driver_1.serial.stopbits = 1  # STOPBITS_ONE
Driver_1.serial.timeout = 0.1
Driver_1.serial.xonxoff = False
Driver_1.serial.rtscts = False
Driver_1.serial.dsrdtr = False
Driver_1.serial.writeTimeout = 0

# Open the communication port
# Driver_1.serial.open()

# Define the motor setup function using minimalmodbus
def motor_setup(Driver_1):
    Driver_1.write_register(53, 20000, functioncode=6)  # EG20000: Sets microstepping to 20,000 steps per revolution
    # Driver_1.write_register(2, 0, functioncode=6)  # IFD: Sets the format of drive responses to decimal
    # Driver_1.write_register(9,0, functioncode=6)  # SP0: Sets the starting position at 0
    Driver_1.write_register(28, 10, functioncode=6)  # AC10: Acceleration
    Driver_1.write_register(29, 15, functioncode=6)  # DE15: Deceleration
    Driver_1.write_register(30, 15, functioncode=6)  # VE15: Velocity
    # Driver_1.write_register(12, 1, functioncode=6)  # ME: Enable Motor
       

# Define the move function using minimalmodbus
def move(Driver_1):
    Driver_1.write_register(31, 60000, functioncode=6)  # FL60000: Moves 3 revs CW
    Driver_1.write_register(31, -120000, functioncode=6)  # FL-120000: Moves 6 revs CCW

    # This section demonstrates the drive's ability to poll immediate position
    # and check status to see if the move is done.
    time.sleep(0.5)
    immediate_position_1 = Driver_1.read_register(5, functioncode=4)  # IP: Immediate position
    time.sleep(1)
    # status = Driver_1.read_register(24, functioncode=4)  # RS: Request status of the drive
    print(f"Immediate Position 1: {immediate_position_1}")
    # print(f"Status: {status}")

try:
    motor_setup(Driver_1)  # Complete motor setup and enable motor
    move(Driver_1)  # Perform the move command

except Exception as e:
    print(f"Error communicating: {str(e)}")

finally:
    # Close the communication port
    Driver_1.serial.close()
