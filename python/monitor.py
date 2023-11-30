import serial
import time
import struct
import argparse

def main():
    # Parse input arguments
    parser = argparse.ArgumentParser(description="Pilot program.")
    parser.add_argument("port", help="The serial port to use.")
    parser.add_argument("-v", "--verbose", action="store_true", help="Print all available serial output.")
    args = parser.parse_args()

    # Define and open serial port
    port_name = args.port
    ser = serial.Serial(port_name, 921600)
    
    count = 0
    while True:
        try:
            line = ser.readline()
            print(line.decode('utf-8'))
        except UnicodeDecodeError:
            for c in line:
                if c == 0xff:
                    print(f"received packet {count}")
                    count += 1
                    break
            continue
        
if __name__ == "__main__":
    main()