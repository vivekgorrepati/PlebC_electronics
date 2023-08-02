import asyncio
import websockets
import serial  # Import the appropriate library for your microcontroller communication

# Define the URL of the WebSocket server
websocket_url = "wss://torusserver-jhybo7-microtica.microtica.rocks"

# Define the serial port settings for your microcontroller
serial_port = "COM11"  # Replace with the actual serial port
baud_rate = 115200  # Replace with the appropriate baud rate
# ser = serial.Serial('COM4', baudrate=9600)

# Create a serial connection to the microcontroller
microcontroller = serial.Serial(serial_port, baud_rate)

async def connect_to_websocket():
    async with websockets.connect(websocket_url) as websocket:
        while True:
            message = await websocket.recv()  # Wait for incoming messages
            print("Received message:", message)
            if(len(message.split('$'))>2):
                print("Received message:", message)
            # Forward the message to the microcontroller
                microcontroller.write(message.encode())  # Encode the message if needed

# Start the WebSocket connection
asyncio.get_event_loop().run_until_complete(connect_to_websocket())
