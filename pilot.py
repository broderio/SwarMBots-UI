import serial
import sys
import time
import struct

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
    if (len(sys.argv) != 2):
        print("Usage: python3 pilot.py <serial port>")
        exit(1)

    port_name = sys.argv[1]
    ser = serial.Serial(port_name, 115200)

    macs = open('macs.txt', 'r')
    mac_list = macs.readlines()
    macs.close()
    
    # Read all available output from serial and print it
    while ser.in_waiting:
        print(ser.readline().decode('utf-8'))
    print("\n\n")

    for mac in mac_list:
        pkt = create_pkt(mac)
        print(f"Sending {len(pkt)} bytes to {mac}")
        ser.write(pkt)
        time.sleep(0.1)
        while ser.in_waiting:
            print(ser.readline().decode('utf-8'))
        
if __name__ == "__main__":
    main()