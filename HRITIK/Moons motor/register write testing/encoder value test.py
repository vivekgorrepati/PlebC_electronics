import minimalmodbus

# Create a Modbus RTU instrument
instrument = minimalmodbus.Instrument('COM15', 1)  # Replace with your device's serial port and slave address

# Set the communication parameters (baudrate, parity, etc.)
instrument.serial.baudrate = 9600
instrument.serial.bytesize = 8
instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout = 0.1 
instrument.serial.xonxoff = False
instrument.serial.rtscts = False
instrument.serial.dsrdtr = False
instrument.serial.writeTimeout = 0
while True  :
    # Specify the Modbus addresses of the MSBs (e.g., registers 5 and 6)
    register_address_start = 4

    # Read the values from two consecutive registers (registers 5 and 6)
    registers_values = instrument.read_registers(register_address_start, 2)

    # Extract the Most Significant Bits (MSBs) from the long integer
    msbs = registers_values[0]

    # Combine the MSBs to form the complete 32-bit integer
    encoder_value = msbs << 16 | registers_values[1]

    # Display the result
    print(f"Encoder value read from Modbus registers {register_address_start} and {register_address_start + 1}: {encoder_value}")
