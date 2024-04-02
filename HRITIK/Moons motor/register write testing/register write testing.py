import time
import minimalmodbus
import serial

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

# # Open the communication port
# instrument.serial.open()

angle = 30
steps_values = (200000/360)*angle
while True :

    try:
        # Write the value 40000 to holding registers 40022 and 40023
        register_address_start = 31  # 40022 is represented as an offset of 1
        values_to_write = [steps_values]
        instrument.write_registers(register_address_start, values_to_write)
        print(f"Values {values_to_write} successfully written to Holding Registers at addresses {register_address_start} and {register_address_start + 1}")
        instrument.write_register(124, 102, functioncode=6)
        time.sleep(0.01)
    # values_to_write = [10000, 10000]
    # instrument.write_registers(register_address_start, values_to_write)
    # print(f"Values {values_to_write} successfully written to Holding Registers at addresses {register_address_start} and {register_address_start + 1}")
        #instrument.write_register(124, 102, functioncode=6)
    except Exception as e:
        print(f"Error writing to holding registers: {str(e)}")

    finally:
        # Close the communication port
        instrument.serial.close()
