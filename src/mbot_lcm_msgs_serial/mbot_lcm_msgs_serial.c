#include "mbot_lcm_msgs_serial.h"

int pose2D_t_deserialize(uint8_t* src, serial_pose2D_t* dest) {
    memcpy(dest, src, sizeof(serial_pose2D_t));
    return 1;
}

int pose2D_t_serialize(serial_pose2D_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_pose2D_t));
    return 1;
}


int mbot_motor_vel_t_deserialize(uint8_t* src, serial_mbot_motor_vel_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_motor_vel_t));
    return 1;
}

int mbot_motor_vel_t_serialize(serial_mbot_motor_vel_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_motor_vel_t));
    return 1;
}


int twist3D_t_deserialize(uint8_t* src, serial_twist3D_t* dest) {
    memcpy(dest, src, sizeof(serial_twist3D_t));
    return 1;
}

int twist3D_t_serialize(serial_twist3D_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_twist3D_t));
    return 1;
}


int mbot_imu_t_deserialize(uint8_t* src, serial_mbot_imu_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_imu_t));
    return 1;
}

int mbot_imu_t_serialize(serial_mbot_imu_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_imu_t));
    return 1;
}


int slam_status_t_deserialize(uint8_t* src, serial_slam_status_t* dest) {
    memcpy(dest, src, sizeof(serial_slam_status_t));
    return 1;
}

int slam_status_t_serialize(serial_slam_status_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_slam_status_t));
    return 1;
}


int mbot_motor_pwm_t_deserialize(uint8_t* src, serial_mbot_motor_pwm_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_motor_pwm_t));
    return 1;
}

int mbot_motor_pwm_t_serialize(serial_mbot_motor_pwm_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_motor_pwm_t));
    return 1;
}


int pose3D_t_deserialize(uint8_t* src, serial_pose3D_t* dest) {
    memcpy(dest, src, sizeof(serial_pose3D_t));
    return 1;
}

int pose3D_t_serialize(serial_pose3D_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_pose3D_t));
    return 1;
}


int timestamp_t_deserialize(uint8_t* src, serial_timestamp_t* dest) {
    memcpy(dest, src, sizeof(serial_timestamp_t));
    return 1;
}

int timestamp_t_serialize(serial_timestamp_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_timestamp_t));
    return 1;
}


int particle_t_deserialize(uint8_t* src, serial_particle_t* dest) {
    memcpy(dest, src, sizeof(serial_particle_t));
    return 1;
}

int particle_t_serialize(serial_particle_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_particle_t));
    return 1;
}


int twist2D_t_deserialize(uint8_t* src, serial_twist2D_t* dest) {
    memcpy(dest, src, sizeof(serial_twist2D_t));
    return 1;
}

int twist2D_t_serialize(serial_twist2D_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_twist2D_t));
    return 1;
}


int mbot_encoders_t_deserialize(uint8_t* src, serial_mbot_encoders_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_encoders_t));
    return 1;
}

int mbot_encoders_t_serialize(serial_mbot_encoders_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_encoders_t));
    return 1;
}


int joy_t_deserialize(uint8_t* src, serial_joy_t* dest) {
    memcpy(dest, src, sizeof(serial_joy_t));
    return 1;
}

int joy_t_serialize(serial_joy_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_joy_t));
    return 1;
}


int point3D_t_deserialize(uint8_t* src, serial_point3D_t* dest) {
    memcpy(dest, src, sizeof(serial_point3D_t));
    return 1;
}

int point3D_t_serialize(serial_point3D_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_point3D_t));
    return 1;
}


int mbot_message_received_t_deserialize(uint8_t* src, serial_mbot_message_received_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_message_received_t));
    return 1;
}

int mbot_message_received_t_serialize(serial_mbot_message_received_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_message_received_t));
    return 1;
}


int mbot_slam_reset_t_deserialize(uint8_t* src, serial_mbot_slam_reset_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_slam_reset_t));
    return 1;
}

int mbot_slam_reset_t_serialize(serial_mbot_slam_reset_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_slam_reset_t));
    return 1;
}

