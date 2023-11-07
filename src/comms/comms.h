#ifndef COMMS_H
#define COMMS_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define ROS_HEADER_LEN  7
#define ROS_FOOTER_LEN  1 
#define ROS_PKG_LEN     ROS_HEADER_LEN + ROS_FOOTER_LEN
#define SYNC_FLAG       0xff
#define VERSION_FLAG    0xfe

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

uint8_t checksum(uint8_t* addends, int len);
int validate_header(uint8_t* header_data);
int validate_message(uint8_t* header_data, uint8_t* msg_data_serialized, uint16_t message_len, char topic_msg_data_checksum);
int encode_msg(uint8_t* MSG, int msg_len, uint16_t TOPIC, uint8_t* ROSPKT, int rospkt_len);

#endif