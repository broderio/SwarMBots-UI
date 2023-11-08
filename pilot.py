# This program will read the MAC addresses stored in macs.txt and store them in a list.
# It will then create a BOTPKT containing a 0 velocity command for each mac address that it finds.
# It will send this packet to the host device over serial so that it will register that mac address as a peer.
# This will enable the user to use the controller in pilot mode to control the robot.

import serial
import sys

def create_pkt(mac):
    # Creates a ROSPKT with a twist_2D_t message with 0 speed
    # Adds mac address and mac address checksum to end
    return
    
if (len(sys.argv) != 2):
    print("Usage: python3 pair.py <serial port>")
    exit(1)

# Open the serial port
port_name = sys.argv[1]
ser = serial.Serial(port_name, 115200)

# Open the file to read from
macs = open('macs.txt', 'r')

# Read the file into a list
mac_list = macs.readlines()

# Close the file
macs.close()

# Loop through the list
for mac in mac_list:
    # Create a BOTPKT with the mac address and a 0 velocity command
    pkt = create_pkt(mac)
    # Send the packet over serial
    ser.write(pkt)