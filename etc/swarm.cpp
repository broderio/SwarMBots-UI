#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <mutex>
#include <termios.h>
#include <unistd.h>

#include "swarm.h"


using std::mutex;
using std::thread;
using std::string;
using std::lock_guard;

Swarm::robot_t::robot_t(){

}

Swarm::robot_t::robot_t(string rob_name){
    //intialize robot
    //pair robot
    //kick off robot thread

}

Swarm::robot_t::robot_t(string rob_name, int mac_addr){
    //initialize robot
    //kick off robot thread

}

Swarm::robot_t::~robot_t(){
    
}

//this needs to be a thread that accesses a queue of things to send, have host
//send back an Ack bit which means needs to be signaled by swarm_thread to continue
int Swarm::serial_send(string mac_addr, uint8_t* packet, uint32_t len){
    ssize_t bytes_written = write(serial_port, packet, len);
    if (bytes_written < 0) {
        perror("Error writing to serial port");
        return 1;
    }

    printf("Sent %ld bytes\n", bytes_written);
}

Swarm::robot_t* Swarm::name_to_rob(string& rob_name){
    lock_guard<mutex> lock(map_lock);
    if (name_to_mac.find(rob_name) == name_to_mac.end()) return nullptr;
    if (mac_to_robot.find(name_to_mac[rob_name]) == mac_to_robot.end()) return nullptr;
    return mac_to_robot[name_to_mac[rob_name]];
}

Swarm::Swarm(){
    port = "";
    num_robots = 0;
    serial_port = -1;
    mac_to_robot = {};
    name_to_mac = {};
}

Swarm::Swarm(string host){
    if (host.find("COM") == string::npos || host.find("COM") != 0 || host.length() > 5){
        //generate an exception and message
    }
    port = host;
    num_robots = 0;
    serial_port = -1;
    mac_to_robot = {};
    name_to_mac = {};
    thread start_swarm(swarm_thread, NULL);
}

Swarm::~Swarm(){
    close(serial_port);
}

void Swarm::swarm_thread(){
    //open com port host is connected to
    //get host mac address and save it
    struct termios tty;
    port.replace(0, 3, "/dev/tty")
    serial_port = open(port, O_RDWR);
    if (serial_port == -1) {
        perror("Error opening serial port");
        return;
    }

    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Error from tcgetattr");
        return;
    }

    tty.c_cflag = B115200; // Set your desired baud rate
    tty.c_cflag |= CS8;   // 8-bit data
    tty.c_cflag |= CLOCAL; // Ignore modem control lines
    tty.c_cflag |= CREAD;  // Enable receiver

    tcflush(serial_port, TCIFLUSH);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        return;
    }

    while (1){
        char buffer[256];
        // Read data from the device, should get ACK
        int bytes_read = read(serial_port, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            printf("Received: %s", buffer);
        }
    }
}

uint8_t* Swarm::command_serializer(float vx, float vy, float wz){
    serial_twist2D_t msg = {
        .vx = vx,
        .vy = vy,
        .wz = wz
    };

    // Initialize variables for packet
    size_t msg_len = sizeof(msg);
    uint8_t* msg_serialized = (uint8_t*)(malloc(msg_len));
    uint8_t* packet = (uint8_t*)(malloc(msg_len + ROS_PKG_LEN));

    // Serialize message and create packet
    twist2D_t_serialize(&msg, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_VEL_CMD, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    return packet;
}

// Gets the odometry
int Swarm::robot_get_odometry(string &rob_name, serial_pose2d_t &odometry){
    Swarm::robot_t* rob = name_to_rob(rob_name);
    lock_guard<mutex> lock(rob->odometry_mutex);
    return rob->odometry;
}

// Gets the IMU
serial_mbot_imu_t Swarm::robot_get_imu(string &rob_name, serial_mbot_imu_t &imu){
    Swarm::robot_t* rob = name_to_rob(rob_name);
    lock_guard<mutex> lock(rob->imu_mutex);
    return rob->imu;
}

// Gets the encoders
serial_mbot_encoders_t Swarm::robot_get_encoders(string &rob_name, serial_mbot_encoders_t &encoders){
    Swarm::robot_t* rob = name_to_rob(rob_name);
    lock_guard<mutex> lock(rob->encoders_mutex);
    return rob->encoders;
}

// Gets the velocity
int Swarm::robot_get_vel(string &rob_name, serial_twist2d_t &velocity_out){
    Swarm::robot_t* rob = name_to_rob(rob_name);
    lock_guard<mutex> lock(rob->velocity_mutex);
    return rob->velocity;
}

// Gets the motor velocity
int Swarm::robot_get_motor_vel(string &rob_name, serial_mbot_motor_vel_t &motor_vel){
    Swarm::robot_t* rob = name_to_rob(rob_name);
    lock_guard<mutex> lock(rob->motor_vel_mutex);
    return rob->motor_vel;
}

// Gets the motor PWM
int Swarm::robot_get_motor_pwm(string &rob_name, serial_mbot_motor_pwm_t &motor_pwm){
    Swarm::robot_t* rob = name_to_rob(rob_name);
    lock_guard<mutex> lock(rob->motor_pwm_mutex);
    return rob->motor_pwm;
}

// Gets the drive mode
int Swarm::robot_get_drive_mode(string &rob_name, int &drive_mode){
    Swarm::robot_t* rob = name_to_rob(rob_name);
    lock_guard<mutex> lock(rob->drive_mode_mutex);
    return this->drive_mode;
}

// Resets the odometry
int Swarm::robot_reset_odometry(string &rob_name){
    Swarm::robot_t* rob = name_to_rob(rob_name);
    lock_guard<mutex> lock(rob->odometry_mutex);
    rob->odometry = serial_pose2d_t();
}

// Resets the encoders
int Swarm::robot_reset_encoders(string &rob_name){
    Swarm::robot_t* rob = name_to_rob(rob_name);
    lock_guard<mutex> lock(rob->encoders_mutex);
    rob->encoders = serial_mbot_encoders_t();
}

// Sets the velocity
int Swarm::robot_set_vel(string &rob_name, float linear, float angular){
    //unsure how to deal 
}

// Sets the drive mode
int Swarm::robot_set_drive_mode(string &rob_name, int mode){

}

// Sets the motor velocity if the drive mode is MODE_MOTOR_VEL_OL
int Swarm::robot_set_motor_vel(string &rob_name, float left, float right){

}

// Sets the motor PWM if the drive mode is MODE_MOTOR_PWM
int Swarm::robot_set_motor_pwm(string &rob_name, float left, float right){
    
}
