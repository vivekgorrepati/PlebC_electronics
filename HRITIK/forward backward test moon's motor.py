import time
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

try:
    # Write the 32-bit integer to two consecutive holding registers (31 and 32)
    register_address_start = 30  # 40022 is represented as an offset of 1
    long_integer_value = 20000  # to move forward write positive steps, for backward write negative steps

    # Split the 32-bit integer into two 16-bit values
    values_to_write = [(long_integer_value >> 16) & 0xFFFF, long_integer_value & 0xFFFF]
    # values_to_write = [long_integer_value & 0xFFFF, (long_integer_value >> 16) & 0xFFFF]
    # values_to_write = [65535,10000]

    # Write the values to the registers
    instrument.write_registers(register_address_start, values_to_write)
    print(f"Values {values_to_write} successfully written to Holding Registers at addresses {register_address_start} and {register_address_start + 1}")

    instrument.write_register(124, 102, functioncode=6)
except Exception as e:
    print(f"Error writing to holding registers: {str(e)}")

finally:
    # Close the communication port
    instrument.serial.close()
