import serial
import re
import sys

def main():
    if (len(sys.argv) != 2):
        print("Usage: python3 pair.py <serial port>")
        exit(1)

    port_name = sys.argv[1]
    ser = serial.Serial(port_name, 115200)

    macs = open('macs.txt', 'a')

    while True:
        line = ser.readline()
        
        mac_address_pattern = r"([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})"
        m = re.search(mac_address_pattern, line)
        if m:
            addr = m.group(0)
            macs.write(addr)
            macs.flush()
            
            print(addr)
            return
        
if __name__ == "__main__":
    main()