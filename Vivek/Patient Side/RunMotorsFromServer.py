import SocketCode
import math as m
import numpy as np
  
import time
import serial

received_message = ""

def OnServer_messageReceived(message):
    global received_message
    received_message = message
    print("Received message:", received_message)
    if(len(message.line.split('$'))>2):
        print("Received message:", received_message)
        #ser.write(bytes(str(received_message), 'utf-8'))


websocket_url = "ws://localhost:3000"  # Replace with your WebSocket server URL
client = SocketCode.WebSocketClient(websocket_url)
client.ws.on_message = OnServer_messageReceived
client.start()


serial_port = 'COM10'  # Replace with your Arduino's serial port
baud_rate = 115200  # Replace with your Arduino's baud rate


# Create the serial connection
#ser = serial.Serial(serial_port, baud_rate)



while True:
    # Do other tasks or user interactions
    # ...

    # Check if a new message is received
    if received_message:
        print("Received message:", received_message)
        # Process the received message or perform any required actions
        # ...

        # Reset the received message variable
        received_message = ""
