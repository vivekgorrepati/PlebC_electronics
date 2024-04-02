import time
import minimalmodbus
import threading
from datetime import datetime

# Initialization
motor1 = minimalmodbus.Instrument('COM15', 1)
motor1.serial.baudrate = 115200
motor1.serial.bytesize = 8
motor1.serial.parity = minimalmodbus.serial.PARITY_NONE
motor1.serial.stopbits = 1
stop_threads = False
motor1.write_register(149,97)




def read_from_register1():
    # while not stop_threads:
    current_steps = 0
    angles_list = [270,-270,30,270,-270,180,270,-270,40,270,-270,270,-270,270,-270,0]
    # try:
    #     value = motor1.read_register(50, functioncode=3)
    #     print("Register 50 value:", value, "Time:", datetime.now().strftime("%H:%M:%S.%f")[:-3])
    # except Exception as e:
    #     print("Error reading register 50 :", e)
    # time.sleep(0.040)
    for serial_number, angle in enumerate(angles_list, start=1):
        try:
            print(serial_number, angle)
            target_steps = int((20000/360) * angle)  # to move forward write positive steps, for backward write negative steps
            steps = target_steps - current_steps
            values_to_write = [(steps >> 16) & 0xFFFF, steps & 0xFFFF]  # Split the 32-bit integer into two 16-bit values

            print( " Before writing time:", datetime.now().strftime("%H:%M:%S.%f")[:-3])
            motor1.write_registers(30, values_to_write)  # Write the values to the registers
            motor1.write_register(124, 102, functioncode=16)  # Send command to control register
            print( " After writing time:", datetime.now().strftime("%H:%M:%S.%f")[:-3])
            current_steps = current_steps + steps  # Updating steps
            
            time.sleep(1)

        except Exception as e:
            print(f"Error writing to holding registers: {str(e)}")


def read_from_register2():
    while not stop_threads:
        try:
            value = motor1.read_register(5, functioncode=3)
            print("Register 5 value:", value, "Time:", datetime.now().strftime("%H:%M:%S.%f")[:-3])
        except Exception as e:
            print("Error reading register 5:", e)
        time.sleep(0.100)



def stop():
    global stop_threads
    stop_threads = True

thread1 = threading.Thread(target=read_from_register1)
thread2 = threading.Thread(target=read_from_register2)

thread1.start()
time.sleep(0.010)
thread2.start()

time.sleep(10)  # Adjust as necessary
stop()

thread1.join()
thread2.join()

motor1.serial.close()
print("Finished. Both threads have been stopped.")
