#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <thread>

#include "swarm.h"

Swarm::robot_t* Swarm::name_to_rob(string& rob_name){
    
}

Swarm::robot_t::robot_t(){

}

Swarm::robot_t::robot_t(string rob_name){

}

Swarm::robot_t::robot_t(string rob_name, int mac_addr){

}

Swarm::robot_t::~robot_t(){
    
}

// Gets the odometry
int Swarm::robot_get_odometry(const string &rob_name, serial_pose2d_t &odometry){
    std::lock_guard<std::mutex> lock(odometry_mutex);
    odometry = this->odometry;
    return rob_error;
}

// Gets the IMU
int Swarm::robot_get_imu(serial_mbot_imu_t &imu){
    std::lock_guard<std::mutex> lock(imu_mutex);
    imu = this->imu;
    return rob_error;
}

// Gets the encoders
int Swarm::robot_t::robot_get_encoders(serial_mbot_encoders_t &encoders){
    std::lock_guard<std::mutex> lock(encoders_mutex);
    encoders = this->encoders;
    return rob_error;
}

// Gets the velocity
int Swarm::robot_t::robot_get_vel(serial_twist2d_t &velocity){
    std::lock_guard<std::mutex> lock(velocity_mutex);
    velocity = this->velocity;
    return rob_error;
}

// Gets the motor velocity
int Swarm::robot_t::robot_get_motor_vel(serial_mbot_motor_vel_t &motor_vel){
    std::lock_guard<std::mutex> lock(motor_vel_mutex);
    motor_vel = this->motor_vel;
    return rob_error;
}

// Gets the motor PWM
int Swarm::robot_t::robot_get_motor_pwm(serial_mbot_motor_pwm_t &motor_pwm){
    std::lock_guard<std::mutex> lock(motor_pwm_mutex);
    motor_pwm = this->motor_pwm;
    return rob_error;
}

// Gets the drive mode
int Swarm::robot_t::robot_get_drive_mode(int &drive_mode){
    std::lock_guard<std::mutex> lock(drive_mode_mutex);
    drive_mode = this->drive_mode;
    return rob_error;
}

// Resets the odometry
int Swarm::robot_t::robot_reset_odometry(){
    std::lock_guard<std::mutex> lock(odometry_mutex);
    this->odometry = serial_pose2d_t();
}

// Resets the encoders
int Swarm::robot_t::robot_reset_encoders(){
    std::lock_guard<std::mutex> lock(encoders_mutex);
    this->encoders = serial_mbot_encoders_t();
}

// Sets the velocity
int Swarm::robot_t::robot_set_vel(float linear, float angular){
    //unsure how to deal 
}

// Sets the drive mode
int Swarm::robot_t::robot_set_drive_mode(int mode){

}

// Sets the motor velocity if the drive mode is MODE_MOTOR_VEL_OL
int Swarm::robot_t::robot_set_motor_vel(float left, float right){

}

// Sets the motor PWM if the drive mode is MODE_MOTOR_PWM
int Swarm::robot_t::robot_set_motor_pwm(float left, float right){
    
}