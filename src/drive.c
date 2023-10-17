#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "mbot_lcm_msgs_serial.h"
#include "mbot_params.h"
#include "comms.h"


uint8_t* command_serializer(float vx, float vy, float wz){
    serial_twist2D_t msg = {
        .vx = vx,
        .vy = vy,
        .wz = wz
    };

    // Initialize variables for packet
    size_t msg_len = sizeof(msg);
    uint8_t* msg_serialized = reinterpret_cast<uint8_t*>(malloc(msg_len));
    uint8_t* packet = reinterpret_cast<uint8_t*>(malloc(msg_len + ROS_PKG_LEN));

    // Serialize message and create packet
    twist2D_t_serialize(&msg, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_VEL_CMD, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    return packet;
}

int main() {
    int serial_port;
    struct termios tty;

    serial_port = open("/dev/ttyS12", O_RDWR);
    if (serial_port == -1) {
        perror("Error opening serial port");
        return 1;
    }

    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Error from tcgetattr");
        return 1;
    }

    tty.c_cflag = B115200; // Set your desired baud rate
    tty.c_cflag |= CS8;   // 8-bit data
    tty.c_cflag |= CLOCAL; // Ignore modem control lines
    tty.c_cflag |= CREAD;  // Enable receiver

    tcflush(serial_port, TCIFLUSH);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        return 1;
    }

    float vx, vy, wz;
    printf("Enter vx value:");
    scanf("%f", &vx);

    vy = 0;

    printf("Enter wz value:");
    scanf("%f", &wz);

    uint8_t* packet = command_serializer(vx, vy, wz);
    
    ssize_t bytes_written = write(serial_port, packet, sizeof(serial_twist2D_t) + ROS_PKG_LEN);
    if (bytes_written < 0) {
        perror("Error writing to serial port");
        return 1;
    }

    printf("Sent %ld bytes\n", bytes_written);
    for (int i = 0; i < sizeof(serial_twist2D_t) + ROS_PKG_LEN; i++) {
        printf("packet[%d]: 0x%x\n", i, packet[i]);
    }

    char buffer[256];
    // Read data from the device, should get ACK
    int bytes_read = read(serial_port, buffer, sizeof(buffer));
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        printf("Received: %s", buffer);
    }

    close(serial_port);

    return 0;
}
