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


    # // add mac address
    # msg_ser[0] = SYNC_FLAG;
    # msg_ser[1] = (uint8_t) (msg_len + ROS_PKG_LEN % 255);
    # msg_ser[2] = (uint8_t) (msg_len + ROS_PKG_LEN >> 8);
    # memcpy(msg_ser + 3, mac_address, MAC_ADDR_LEN);
    # uint8_t cs0_addends[MAC_ADDR_LEN + 2] = {msg_ser[1], msg_ser[2]};
    # msg_ser[9] = checksum(cs0_addends, MAC_ADDR_LEN + 2);

    # // add ROS packet header
    # msg_ser[10] = SYNC_FLAG;
    # msg_ser[11] = VERSION_FLAG;
    # msg_ser[12] = (uint8_t) (msg_len % 255);
    # msg_ser[13] = (uint8_t) (msg_len >> 8);
    # uint8_t cs1_addends[2] = {msg_ser[12], msg_ser[13]};
    # msg_ser[14] = checksum(cs1_addends, 2);

    # // add topic and message
    # msg_ser[15] = (uint8_t) (topic % 255);
    # msg_ser[16] = (uint8_t) (topic >> 8);
    # memcpy(msg_ser + 17, msg, msg_len);
    # uint8_t cs2_addends[msg_len + 2];
    # cs2_addends[0] = msg_ser[15];
    # cs2_addends[1] = msg_ser[16];
    # memcpy(cs2_addends + 2, msg, msg_len);
    # msg_ser[17 + msg_len] = checksum(cs2_addends, msg_len + 2);

def create_pkt(mac: str) -> bytes:
    msg = bytearray()
    msg.extend(struct.pack('B', SYNC_FLAG)) # 8 bit unsigned integer, msg[0]
    msg.extend(reversed(struct.pack('H', 16))) # 16 bit unsigned integer, msg[1:3]
    msg.extend(mac_str_to_bytes(mac)) # 6 byte MAC address, msg[3:9]
    msg.extend(struct.pack('B', checksum(msg[1:9]))) # 8 bit unsigned integer, msg[9]
    
    msg.extend(struct.pack('B', SYNC_FLAG)) # 8 bit unsigned integer, msg[10]
    msg.extend(struct.pack('B', VERSION_FLAG)) # 8 bit unsigned integer, msg[11]
    msg.extend(reversed(struct.pack('H', 8))) # 16 bit unsigned integer, msg[12:14]
    msg.extend(struct.pack('B', checksum(msg[12:14]))) # 8 bit unsigned integer, msg[14]
    
    msg.extend(reversed(struct.pack('H', MBOT_TIMESYNC))) # 16 bit unsigned integer, msg[15:17]
    msg.extend(reversed(struct.pack('Q', int(time.time() * 1000000)))) # 64 bit unsigned integer, msg[17:25]
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

    for mac in mac_list:
        pkt = create_pkt(mac)
        ser.write(pkt)
        
if __name__ == "__main__":
    main()