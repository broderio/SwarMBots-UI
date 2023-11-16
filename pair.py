import serial
import re
import argparse

def main():
    parser = argparse.ArgumentParser(description="Read MAC addresses from a serial port.")
    parser.add_argument("port", help="The serial port to read from.")
    parser.add_argument("-c", "--clean", action="store_true", help="Overwrite the macs.txt file with new MAC addresses.")
    args = parser.parse_args()

    port_name = args.port
    ser = serial.Serial(port_name, 921600)
    
    file_mode = 'w' if args.clean else 'a'
    macs = open('macs.txt', file_mode)

    while True:
        try:
            line = ser.readline().decode('utf-8')
        except UnicodeDecodeError:
            continue
        
        mac_address_pattern = r"([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})"
        m = re.search(mac_address_pattern, line)
        if m:
            addr = m.group(0)
            macs.write(addr + '\n')
            macs.flush()
            
            print(addr)
            return
        
if __name__ == "__main__":
    main()