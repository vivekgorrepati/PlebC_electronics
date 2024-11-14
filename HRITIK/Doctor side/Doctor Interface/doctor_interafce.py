import tkinter as tk
from tkinter import Label, PhotoImage
import serial
import minimalmodbus

from PIL import ImageTk, Image


# Configure the serial settings (adjust these according to your setup)
PORT = 'COM15'
BAUDRATE = 115200
TIMEOUT = 0.4

# Instrument configurations for the three motors
instrument1 = minimalmodbus.Instrument(PORT, 1)
instrument1.serial.baudrate = BAUDRATE
instrument1.serial.timeout = TIMEOUT
instrument1.serial.parity = serial.PARITY_NONE
instrument1.serial.stopbits = 1
instrument1.serial.bytesize = 8

instrument2 = minimalmodbus.Instrument(PORT, 2)
instrument2.serial.baudrate = BAUDRATE
instrument2.serial.timeout = TIMEOUT
instrument2.serial.parity = serial.PARITY_NONE
instrument2.serial.stopbits = 1
instrument2.serial.bytesize = 8

instrument3 = minimalmodbus.Instrument(PORT, 3)
instrument3.serial.baudrate = BAUDRATE
instrument3.serial.timeout = TIMEOUT
instrument3.serial.parity = serial.PARITY_NONE
instrument3.serial.stopbits = 1
instrument3.serial.bytesize = 8

# Function to send angles from slider directly to motors
def update_motor1(val):
    try:
        angle1 = int(val)
        instrument1.write_register(4, angle1 * 2, functioncode=6)
    except Exception as e:
        print(f"Failed to send angle to motor 1: {e}")

def update_motor2(val):
    try:
        angle2 = int(val)
        instrument2.write_register(4, angle2 * 27, functioncode=6)  # Adjust multiplication for motor 2
    except Exception as e:
        print(f"Failed to send angle to motor 2: {e}")

def update_motor3(val):
    try:
        angle3 = int(val)
        instrument3.write_register(4, angle3, functioncode=6)
    except Exception as e:
        print(f"Failed to send angle to motor 3: {e}")

# Tkinter GUI setup
root = tk.Tk()
root.title("PlebC - TORUS ")
# root.geometry('600x600')
# Set the size and background color
root.geometry('500x600')
root.configure(bg='white')

# Add the company logo (adjust the path to your logo)
# logo = PhotoImage('logo.jpg')  # Replace with actual file path
# logo_label = Label(root, image=logo, bg='white')
# logo_label.pack(pady=10)

img = Image.open('logo.jpg')
resized_img = img.resize((150,70))
img = ImageTk.PhotoImage(resized_img)
img_lable = Label(root,image = img)
img_lable.pack()


# Add product and company name
title = Label(root, text="TORUS", font=("Arial", 24, "bold"), fg='#54329e', bg='white')
title.pack(pady=5)

# company_name = Label(root, text="PlebC", font=("Arial", 16), fg='#54329e', bg='white')
# company_name.pack(pady=5)

# Create sliders for each motor with labels
slider_motor1 = tk.Scale(root, from_=0, to=360, orient=tk.HORIZONTAL, label="Link 1 Angle", 
                         length=400, command=update_motor1, fg='#54329e', bg='white', 
                         troughcolor='#54329e')
slider_motor1.pack(pady=10)

slider_motor2 = tk.Scale(root, from_=0, to=360, orient=tk.HORIZONTAL, label="Link 2 Angle", 
                         length=400, command=update_motor2, fg='#54329e', bg='white', 
                         troughcolor='#54329e')
slider_motor2.pack(pady=10)

slider_motor3 = tk.Scale(root, from_=0, to=360, orient=tk.HORIZONTAL, label="Link 3 Angle", 
                         length=400, command=update_motor3, fg='#54329e', bg='white', 
                         troughcolor='#54329e')
slider_motor3.pack(pady=10)

# Run the GUI loop
root.mainloop()
