#ifndef COMMS_H
#define COMMS_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "msgtypes.h"

#define ROS_HEADER_LEN  7
#define ROS_FOOTER_LEN  1 
#define ROS_PKG_LEN     ROS_HEADER_LEN + ROS_FOOTER_LEN
#define SYNC_FLAG       0xff
#define VERSION_FLAG    0xfe
#define MAC_ADDR_LEN    6

enum message_topics{
    MBOT_TIMESYNC = 201, 
    MBOT_ODOMETRY = 210, 
    MBOT_ODOMETRY_RESET = 211,
    MBOT_VEL_CMD = 214,
    MBOT_IMU = 220,
    MBOT_ENCODERS = 221,
    MBOT_ENCODERS_RESET = 222,
    MBOT_MOTOR_PWM_CMD = 230,
    MBOT_MOTOR_VEL_CMD = 231,
    MBOT_MOTOR_VEL = 232,
    MBOT_MOTOR_PWM = 233,
    MBOT_VEL = 234
};

typedef struct __attribute__((__packed__)) packets_wrapper_t {
    serial_mbot_encoders_t encoders;
    serial_pose2D_t odom;
    serial_mbot_imu_t imu;
    serial_twist2D_t mbot_vel;
    serial_mbot_motor_vel_t motor_vel;
    serial_mbot_motor_pwm_t motor_pwm;
} packets_wrapper_t;

uint8_t checksum(uint8_t* addends, int len);

#ifdef __cplusplus
extern "C" {
#endif
void read_mac_address(int serial_port, uint8_t* mac_address, uint16_t* pkt_len);
void read_header(int serial_port, uint8_t* header_data);
void read_message(int serial_port, uint8_t* msg_data_serialized, uint16_t message_len, char* topic_msg_data_checksum);
int validate_header(uint8_t* header_data);
int validate_message(uint8_t* msg_data_serialized, uint16_t message_len, char topic_msg_data_checksum);
void encode_msg(uint8_t* msg, int msg_len, uint16_t topic, uint8_t mac_address[6], uint8_t* msg_ser, int msg_ser_len);
#ifdef __cplusplus
}
#endif

#endif