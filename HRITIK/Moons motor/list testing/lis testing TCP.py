from pyModbusTCP.client import ModbusClient
import time

# Create a Modbus TCP client
client = ModbusClient(host="10.10.10.10", port=502)  # Replace with your device's IP address

# Connect to the Modbus server
if not client.open():
    print("Error connecting to Modbus server")
    exit(1)

# Initialize current steps
current_steps = 0

# Define the angles list
angles_list = [270, -270, 30, 270, -270, 180, 270, -270, 40, 270, -270, 270, -270, 270, -270, 0]

# Iterate through the angles
for serial_number, angle in enumerate(angles_list, start=1):
    try:
        # Get the current time
        current_time = time.strftime("%H:%M:%S.%f")[:-3]
        print(f"{serial_number} {angle} time: {current_time}")

        # Calculate target steps
        target_steps = int((20000 / 360) * angle)
        steps = target_steps - current_steps
        values_to_write = [(steps >> 16) & 0xFFFF, steps & 0xFFFF]  # Split the 32-bit integer into two 16-bit values

        # Write values to registers (adapt this part based on your specific use case)
        client.write_multiple_registers(30, values_to_write)
        client.write_single_register(124,102)

        # Update current_steps if needed
        current_steps = current_steps + steps  # Updating steps

        # Add a delay (e.g., 300 milliseconds) for real-time updates
        time.sleep(0.3)

    except Exception as e:
        print(f"Error: {e}")

# Close the Modbus TCP client
client.close()
