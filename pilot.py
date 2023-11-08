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
    msg.extend(reversed(struct.pack('B', SYNC_FLAG))) # 8 bit unsigned integer, msg[0]
    msg.extend(struct.pack('B', VERSION_FLAG)) # 8 bit unsigned integer, msg[1]
    msg.extend(reversed(struct.pack('H', 8))) # 16 bit unsigned integer, msg[2:4]
    msg.extend(reversed(struct.pack('B', checksum(msg[2:4])))) # 8 bit unsigned integer, msg[4]
    msg.extend(reversed(struct.pack('H', MBOT_TIMESYNC))) # 16 bit unsigned integer, msg[5:7]
    msg.extend(reversed(struct.pack('Q', int(time.time() * 1000000)))) # 64 bit unsigned integer, msg[7:15]
    msg.extend(reversed(struct.pack('B', checksum(msg[7:15])))) # 8 bit unsigned integer, msg[15]
    msg.extend(mac_str_to_bytes(mac)) # 6 byte MAC address, msg[16:22]
    msg.extend(reversed(struct.pack('B', checksum(msg[16:22])))) # 8 bit unsigned integer, msg[28]
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

    for mac in mac_list:
        pkt = create_pkt(mac)
        ser.write(pkt)
        
if __name__ == "__main__":
    main()