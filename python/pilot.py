import serial
import time
import struct
import argparse

SYNC_FLAG = 0xff # beginning of packet sync flag
VERSION_FLAG = 0xfe # version flag compatible with ROS2
MBOT_TIMESYNC = 201 # Timesync topic number

def checksum(addends: list[int]) -> int:
    return 255 - (sum(addends) % 256)

def mac_str_to_bytes(mac: str)->bytes:
    return bytes.fromhex(mac.replace(':', ''))

def create_pkt(mac: str) -> bytes:
    msg = bytearray()
    msg.extend(struct.pack('B', SYNC_FLAG)) # 8 bit unsigned integer, msg[0]
    msg.extend(struct.pack('<H', 16)) # 16 bit unsigned integer, msg[1:3]
    msg.extend(mac_str_to_bytes(mac)) # 6 byte MAC address, msg[3:9]
    
    msg.extend(struct.pack('B', SYNC_FLAG)) # 8 bit unsigned integer, msg[10]
    msg.extend(struct.pack('B', VERSION_FLAG)) # 8 bit unsigned integer, msg[11]
    msg.extend(struct.pack('<H', 8)) # 16 bit unsigned integer, msg[12:14]
    msg.extend(struct.pack('B', checksum(msg[12:14]))) # 8 bit unsigned integer, msg[14]
    
    msg.extend(struct.pack('<H', MBOT_TIMESYNC)) # 16 bit unsigned integer, msg[15:17]
    msg.extend(struct.pack('<Q', int(time.time() * 1000000))) # 64 bit unsigned integer, msg[17:25]
    msg.extend(struct.pack('B', checksum(msg[15:25]))) # 8 bit unsigned integer, msg[25]
    return msg

def main():
    # Parse input arguments
    parser = argparse.ArgumentParser(description="Pilot program.")
    parser.add_argument("port", help="The serial port to use.")
    parser.add_argument("-v", "--verbose", action="store_true", help="Print all available serial output.")
    args = parser.parse_args()

    # Define and open serial port
    port_name = args.port
    ser = serial.Serial(port_name, 921600)

    # Read MAC addresses from file
    macs = open('macs.txt', 'r')
    mac_list = macs.readlines()
    macs.close()
    
    # Read all available output from serial and print it
    if args.verbose:
        while ser.in_waiting:
            print(ser.readline().decode('utf-8'))
        print("\n\n")

    # Send packets to each MAC address
    for mac in mac_list:
        if (mac[0] == '#'):
            continue
        
        pkt = create_pkt(mac)
        print(f"Sending {len(pkt)} bytes to {mac}")
        ser.write(pkt)
        
        time.sleep(0.1)
        
        # Print terminal output
        if args.verbose:
            while ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8')
                    print(line)
                except UnicodeDecodeError:
                    continue
        
if __name__ == "__main__":
    main()