#https://medium.com/@simone.b/using-modbus-with-python-a-practical-guide-for-implementation-770ac350ec0d



import minimalmodbus
import time

# Create a Modbus RTU instrument
instrument = minimalmodbus.Instrument('COM15', slaveaddress=1)  # Replace with your device's serial port and slave address

# Set the communication parameters (baudrate, parity, etc.)
instrument.serial.baudrate = 9600
instrument.serial.parity = minimalmodbus.serial.PARITY_NONE

# Add more configuration settings if needed
instrument.serial.bytesize = 8  # EIGHTBITS
instrument.serial.stopbits = 1  # STOPBITS_ONE
instrument.serial.timeout = 0.1
instrument.serial.xonxoff = False
instrument.serial.rtscts = False
instrument.serial.dsrdtr = False
instrument.serial.writeTimeout = 0

# register_address = 124
# value_to_write = 216
instrument.write_register(124, 150, functioncode=6)
print(f"Value {150} successfully written to Holding Register at {124}")
time.sleep(10)
instrument.write_register(124, 216, functioncode=6)
print(f"Value {216} successfully written to Holding Register at {124}")


# while True :
#     print(instrument.read_register(5, functioncode=3))



