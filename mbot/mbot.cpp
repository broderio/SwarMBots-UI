#include "mbot.hpp"
#include <cstring>
#include <type_traits>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "mbot_lcm_msgs_serial.h"
#include "mbot_params.h"
#include "comms.h"

using mac_address_t = uint8_t[MAC_LENGTH];

// template <typename T>
// struct is_queue : std::false_type {};

// template <typename T, typename Container>
// struct is_queue<std::queue<T, Container>> : std::true_type {};

/* thread_safe_t member function definitions */

template <typename T>
thread_safe_t<T>::thread_safe_t()
{
    data = T();
}

template <typename T>
thread_safe_t<T>::~thread_safe_t() {}

template <typename T>
T thread_safe_t<T>::get()
{
    std::lock_guard<std::mutex> lock(mtx);
    return data;
}

template <typename T>
void thread_safe_t<T>::set(T data)
{
    std::lock_guard<std::mutex> lock(mtx);
    this->data = data;
}

/* mbot member function definitions */

// Set static bool false for initial constructor
bool mbot::th_running = false;

// Constructor for mbot class. This function is not thread safe! Mbots should not be instantiated concurrently.
mbot::mbot(const std::string &name, const mac_address_t mac_address, const mbot_params_t &params)
{
    this->name = name;
    std::memcpy(this->mac_address, mac_address, MAC_LENGTH);
    this->params = params;
    this->is_alive = true;

    // Add the robot to the swarm
    mbots.insert(std::pair<std::string, mbot *>(mac_str, this));

    mbot m(name, mac_address, params);

    data_th_handle = std::thread(&mbot::data_th, this); //thread for each robot to update itself

    // Start the thread
    if (!th_running)
    {
        th_running = true;
        mbot_th_handle = std::thread(&mbot::mbot_th); //starts send_thread implicitly
    }
}

// Destructor for mbot class. This function is not thread safe! Mbots should be not be freed concurrently.
mbot::~mbot()
{
    // Remove the robot from the swarm
    std::string mac_str = mac_to_string(mac_address);
    mbots.erase(mac_str);

    // Stop the thread if there are no more robots
    if (mbots.size() == 0)
    {
        th_running = false;
        //need a clean way to terminate the threads
        //thinking make a thread safe counter and use a thread safe "is alive"
    }
}

// Converts uin8_t[MAC_LENGTH] to a string where each 2 bytes are separated by semicolons
std::string mbot::mac_to_string(const mac_address_t mac_address)
{
    std::stringstream ss;
    for (int i = 0; i < 12; ++i)
    {
        if (i > 0 && i % 2 == 0)
        {
            ss << ':';
        }
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(mac_address[i]);
    }
    return ss.str();
}

serial_twist2D_t mbot::get_robot_vel(){
    return this->robot_vel.get();
} 

serial_mbot_imu_t mbot::get_imu(){
    return this->imu.get();
} 

serial_mbot_motor_vel_t mbot::get_motor_vel(){
    return this->motor_vel.get();
} 

serial_twist2D_t mbot::get_robot_vel_goal(){
    std::lock_guard<std::mutex> lock(this->robot_vel_mutex);
    return this->robot_vel_goal;
} 

serial_mbot_motor_vel_t mbot::get_motor_vel_goal(){
    std::lock_guard<std::mutex> lock(this->motor_vel_mutex);
    return this->motor_vel_goal;
} 

void mbot::set_robot_vel_goal(float vx, float vy, float wz) {
    this->robot_vel_mutex.lock();
    this->robot_vel_goal.vx = vx;
    this->robot_vel_goal.vy = vy;
    this->robot_vel_goal.wz = wz;

    // Initialize variables for packet
    size_t msg_len = sizeof(this->robot_vel_goal);
    uint8_t* msg_serialized = (uint8_t*)(malloc(msg_len));
    uint8_t* packet = (uint8_t*)(malloc(msg_len + ROS_PKG_LEN));

    // Serialize message and create packet
    twist2D_t_serialize(&this->robot_vel_goal, msg_serialized);
    this->robot_vel_mutex.unlock();
    encode_msg(msg_serialized, msg_len, MBOT_VEL_CMD, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    uint8_t len = sizeof(robot_vel_goal) + ROS_PKG_LEN;

    //add to the send queue TODO: figure out how we specify MAC for host to send to
    this->send_mutex.lock();
    this->send_queue.push(packet);
    this->send_queue.push(&len);
    this->send_mutex.unlock();

    //alert the send thread there is work to do
    this->send_cv.notify_one();
}

void mbot::set_motor_vel_goal(float a, float b, float c = 0.0f) {
    //TODO: make sure I'm doing this right
    this->motor_vel_mutex.lock();
    this->motor_vel_goal.velocity[0] = a;
    this->motor_vel_goal.velocity[1] = b;
    this->motor_vel_goal.velocity[2] = c;

    // Initialize variables for packet
    size_t msg_len = sizeof(this->motor_vel_goal);
    uint8_t* msg_serialized = (uint8_t*)(malloc(msg_len));
    uint8_t* packet = (uint8_t*)(malloc(msg_len + ROS_PKG_LEN));

    // Serialize message and create packet
    mbot_motor_vel_t_serialize(&this->motor_vel_goal, msg_serialized);
    motor_vel_mutex.unlock();
    encode_msg(msg_serialized, msg_len, MBOT_MOTOR_VEL_CMD, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    uint8_t len = sizeof(motor_vel_goal) + ROS_PKG_LEN;

    //add to the send queue TODO: figure out how we specify MAC for host to send to
    this->send_mutex.lock();
    this->send_queue.push(packet);
    this->send_queue.push(&len);
    this->send_mutex.unlock();

    //alert the send thread there is work to do
    this->send_cv.notify_one();
}

serial_mbot_motor_pwm_t mbot::get_motor_pwm(){
    std::lock_guard<std::mutex> lock(this->motor_pwm_mutex);
    return this->motor_pwm.get();
} 

void mbot::set_motor_pwm(float a, float b, float c = 0.0f) {
    serial_mbot_motor_pwm_t mbot_pwm;
    mbot_pwm.pwm[0] = a;
    mbot_pwm.pwm[1] = b;
    mbot_pwm.pwm[2] = c;
    this->motor_pwm.set(mbot_pwm);
    // Initialize variables for packet
    size_t msg_len = sizeof(mbot_pwm);
    uint8_t* msg_serialized = (uint8_t*)(malloc(msg_len));
    uint8_t* packet = (uint8_t*)(malloc(msg_len + ROS_PKG_LEN));

    // Serialize message and create packet
    mbot_motor_pwm_t_serialize(&mbot_pwm, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_MOTOR_PWM_CMD, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    uint8_t len = sizeof(mbot_pwm) + ROS_PKG_LEN;

    //add to the send queue TODO: figure out how we specify MAC for host to send to
    send_mutex.lock();
    send_queue.push(packet);
    send_queue.push(&len);
    send_mutex.unlock();

    //alert the send thread there is work to do
    send_cv.notify_one();
}

serial_pose2D_t mbot::get_odom() {
    return this->odom.get();
}

serial_mbot_encoders_t mbot::get_encoders() {
    return this->encoders.get();
}

void mbot::set_odom(float x, float y, float theta) {
    serial_pose2D_t mbot_odom;
    mbot_odom.x = x;
    mbot_odom.y = y;
    mbot_odom.theta = theta;
    this->odom.set(mbot_odom);
    // Initialize variables for packet
    size_t msg_len = sizeof(mbot_odom);
    uint8_t* msg_serialized = (uint8_t*)(malloc(msg_len));
    uint8_t* packet = (uint8_t*)(malloc(msg_len + ROS_PKG_LEN));

    // Serialize message and create packet
    pose2D_t_serialize(&mbot_odom, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_ODOMETRY, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    uint8_t len = sizeof(mbot_odom) + ROS_PKG_LEN;

    //add to the send queue TODO: figure out how we specify MAC for host to send to
    send_mutex.lock();
    send_queue.push(packet);
    send_queue.push(&len);
    send_mutex.unlock();

    //alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::reset_odom() {
    serial_pose2D_t mbot_odom;
    mbot_odom.x = 0;
    mbot_odom.y = 0;
    mbot_odom.theta = 0;
    this->odom.set(mbot_odom);
    // Initialize variables for packet
    size_t msg_len = sizeof(mbot_odom);
    uint8_t* msg_serialized = (uint8_t*)(malloc(msg_len));
    uint8_t* packet = (uint8_t*)(malloc(msg_len + ROS_PKG_LEN));

    // Serialize message and create packet
    pose2D_t_serialize(&mbot_odom, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_ODOMETRY_RESET, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    uint8_t len = sizeof(mbot_odom) + ROS_PKG_LEN;

    //add to the send queue TODO: figure out how we specify MAC for host to send to
    send_mutex.lock();
    send_queue.push(packet);
    send_queue.push(&len);
    send_mutex.unlock();

    //alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::set_encoders(int a, int b, int c = 0) {
    serial_mbot_encoders_t mbot_encoders;
    mbot_encoders.ticks[0] = a;
    mbot_encoders.ticks[1] = b;
    mbot_encoders.ticks[2] = c;
    this->encoders.set(mbot_encoders);
    // Initialize variables for packet
    size_t msg_len = sizeof(mbot_encoders);
    uint8_t* msg_serialized = (uint8_t*)(malloc(msg_len));
    uint8_t* packet = (uint8_t*)(malloc(msg_len + ROS_PKG_LEN));

    // Serialize message and create packet
    mbot_encoders_t_serialize(&mbot_encoders, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_ENCODERS, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    uint8_t len = sizeof(mbot_encoders) + ROS_PKG_LEN;

    //add to the send queue TODO: figure out how we specify MAC for host to send to
    send_mutex.lock();
    send_queue.push(packet);
    send_queue.push(&len);
    send_mutex.unlock();

    //alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::reset_encoders() {
    serial_mbot_encoders_t mbot_encoders;
    mbot_encoders.ticks[0] = 0;
    mbot_encoders.ticks[1] = 0;
    mbot_encoders.ticks[2] = 0;
    this->encoders.set(mbot_encoders);
    // Initialize variables for packet
    size_t msg_len = sizeof(mbot_encoders);
    uint8_t* msg_serialized = (uint8_t*)(malloc(msg_len));
    uint8_t* packet = (uint8_t*)(malloc(msg_len + ROS_PKG_LEN));

    // Serialize message and create packet
    mbot_encoders_t_serialize(&mbot_encoders, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_ENCODERS_RESET, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    uint8_t len = sizeof(mbot_encoders) + ROS_PKG_LEN;

    //add to the send queue TODO: figure out how we specify MAC for host to send to
    send_mutex.lock();
    send_queue.push(packet);
    send_queue.push(&len);
    send_mutex.unlock();

    //alert the send thread there is work to do
    send_cv.notify_one();
}

//Reads all incoming data on USB
void mbot::mbot_th() {
    //open com port host is connected to
    //get host mac address and save it
    struct termios tty;
    port_name.replace(0, 3, "/dev/tty");
    serial_port = open(port_name.c_str(), O_RDWR);
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

    //start the write thread now that the serial port is open
    send_th_handle = std::thread(&mbot::send_th);
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

void mbot::send_th(){
    while (1){
        uint8_t* packet;
        uint8_t len;
        { //scope for which we need the lock on the queue
            std::unique_lock<std::mutex> queue_lock(send_mutex);        
            while (send_queue.empty()){
                send_cv.wait(queue_lock);
            }
            packet = send_queue.front();
            send_queue.pop();
            len = *send_queue.front();
            send_queue.pop();
        }
        ssize_t bytes_written = write(serial_port, packet, len);
        if (bytes_written < 0) {
            perror("Error writing to serial port");
        }
        else{
            printf("Sent %ld bytes\n", bytes_written);
            for (uint32_t i = 0; i < sizeof(serial_twist2D_t) + ROS_PKG_LEN; i++) {
                printf("packet[%d]: 0x%x\n", i, packet[i]);
            }
        }
    }
}

void data_th(){
    //need to figure out how best to handle this...
}