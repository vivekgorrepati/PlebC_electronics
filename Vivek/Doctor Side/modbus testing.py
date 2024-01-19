#https://medium.com/@simone.b/using-modbus-with-python-a-practical-guide-for-implementation-770ac350ec0d



import minimalmodbus

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

# Perform Modbus operations here
# Read a single holding register at address 40005
value = instrument.read_register(5,2)
print("Read Holding Register at 40005:", value)

# Close the connection
instrument.serial.close()