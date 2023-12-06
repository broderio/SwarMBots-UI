import serial
import socket
import threading
import struct
import argparse

def get_packet_serial(ser: serial.Serial) -> bytes:
    # Wait for the start of a packet
    trigger = 0x00
    while trigger != 0xff:
        trigger = ser.read()[0]

    # Read the length of the packet and its MAC address
    length = ser.read(2)
    mac = ser.read(6)

    # Validate length
    length_int = struct.unpack('<H', length)[0]
    if length_int != 204:
        return None
    
    # Read the rest of the packet
    packet = ser.read(204)

    # Return reassembled packet
    return bytes([trigger]) + length + mac + packet

def recv_exact(connection: socket.socket, length: int) -> bytes:
    data = b''
    while len(data) < length:
        chunk = connection.recv(length - len(data))
        if not chunk:  # Connection closed
            return None
        data += chunk
    return data

def get_packet_socket(connection: socket.socket) -> bytes:
    # Wait for the start of a packet
    trigger = 0x00
    while trigger != 0xff:
        data = connection.recv(1)
        if not data:  # Connection closed
            return None
        trigger = data[0]

    # Read the length of the packet and its MAC address
    length = recv_exact(connection, 2)
    if length is None:  # Connection closed
        return None
    mac = recv_exact(connection, 6)
    if mac is None:  # Connection closed
        return None

    # Validate length
    length_int = struct.unpack('<H', length)[0]

    # Read the rest of the packet
    packet = recv_exact(connection, length_int)
    if packet is None:  # Connection closed
        return None

    # Return reassembled packet
    return bytes([trigger]) + length + mac + packet

def serial_to_socket(connection, ser):
    while True:
        data = get_packet_serial(ser)
        if data:
            connection.sendall(data)

def socket_to_serial(connection, ser):
    while True:
        data = get_packet_socket(connection)
        ser.write(data)
    
def main():
    # Parse input arguments
    parser = argparse.ArgumentParser(description="UART Socket program.")
    parser.add_argument("port", help="The serial port to use.")
    args = parser.parse_args()

    # Define and open serial port
    port_name = args.port
    ser = serial.Serial(port_name, 921600)

    # Create a socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind the socket to the port
    server_address = ('127.0.0.1', 9002)
    sock.bind(server_address)
    
    # Listen for incoming connections
    sock.listen(1)

    print('waiting for a connection')
    connection, client_address = sock.accept()
    print('connection from', client_address)

    threading.Thread(target=serial_to_socket, args=(connection, ser)).start()
    threading.Thread(target=socket_to_serial, args=(connection, ser)).start()

        
if __name__ == "__main__":
    main()