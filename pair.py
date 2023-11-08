# The user will specify a serial port as an argument
# A client device will be plugged into the USB port and send data over serial
# This program will parse that data looking for a MAC address
# Once it finds a MAC address it will add it to the file macs.txt, print, and exit
# Plug another client device into the USB port and repeat

import serial
import re
import sys

if (len(sys.argv) != 2):
    print("Usage: python3 pair.py <serial port>")
    exit(1)

# Open the serial port
port_name = sys.argv[1]
ser = serial.Serial(port_name, 115200)

# Open the file to append to
macs = open('macs.txt', 'a')

while True:
    # Read a line from the serial port
    line = ser.readline()
    # Check if the line contains a MAC address
    m = re.match(r"(?:[0-9a-fA-F]:?){12}", line)
    if m:
        # If it does, save the MAC address
        addr = m.group(0)
        macs.write(addr)
        # Flush the file to make sure the data is written
        macs.flush()
        # Print the line to the screen
        print(addr)
        exit(0)