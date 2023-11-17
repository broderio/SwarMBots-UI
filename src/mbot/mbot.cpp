#include <cstring>
#include <type_traits>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <chrono>
#include <thread>
#include <iostream>

#include "mbot.h"

// template <typename T>
// struct is_queue : std::false_type {};

// template <typename T, typename Container>
// struct is_queue<std::queue<T, Container>> : std::true_type {};

/* thread_safe_t member function definitions */

template <typename T>
mbot::thread_safe_t<T>::thread_safe_t()
{
    data = T();
}

template <typename T>
mbot::thread_safe_t<T>::thread_safe_t(T data)
{
    this->data = data;
}

template <typename T>
mbot::thread_safe_t<T>::~thread_safe_t() {}

template <typename T>
T mbot::thread_safe_t<T>::get()
{
    std::lock_guard<std::mutex> lock(mtx);
    return data;
}

template <typename T>
void mbot::thread_safe_t<T>::set(T data)
{
    std::lock_guard<std::mutex> lock(mtx);
    this->data = data;
}

/* packet defintiion */

mbot::packet_t::packet_t()
{
    data = nullptr;
    length = 0;
}

mbot::packet_t::packet_t(const mbot::packet_t& other){
    this->data = new uint8_t[other.length];
    for (int i = 0; i < other.length; i++) {
        this->data[i] = other.data[i];
    }
    this->length = other.length;
}

mbot::packet_t::~packet_t()
{
    if (data != nullptr)
    {
        delete[] data;
    }
}

mbot::packet_t& mbot::packet_t::operator=(const mbot::packet_t& other){
    this->data = new uint8_t[other.length];
    for (int i = 0; i < other.length; i++) {
        this->data[i] = other.data[i];
    }
    this->length = other.length;
    return *this;
}

/* mbot member function definitions */

// Set static bool false for initial constructor
mbot::thread_safe_t<bool> mbot::running(false);
std::unordered_map<std::string, mbot *> mbot::mbots;
mbot::thread_safe_t<int> mbot::num_mbots;
std::thread mbot::mbot_th_handle;
int mbot::serial_port;
std::string mbot::port_name;

std::thread mbot::send_th_handle;
std::mutex mbot::send_mutex;
std::queue<mbot::packet_t> mbot::send_queue;
std::condition_variable mbot::send_cv;

// Default constructor for mbot class
mbot::mbot()
{
    this->name = "mbot";
    this->is_alive = true;
    this->params = mbot_params_t();
}

// Constructor for mbot class. This function is not thread safe! Mbots should not be instantiated concurrently.
mbot::mbot(const std::string &name, const mac_address_t mac_address, const std::string &port_in, const mbot_params_t &params)
{
    this->name = name;
    std::memcpy(this->mac_address, mac_address, MAC_ADDR_LEN);
    this->params = params;
    this->is_alive = true;
    // Add the robot to the swarm
    mbots.insert(std::pair<std::string, mbot *>(mac_str, this));

    // Increment the number of robots
    num_mbots.set(num_mbots.get() + 1);

    // Start the thread
    if (!running.get())
    {
        //TODO: read the txt file containing the mac addresses of the clients and initialize map
        port_name = port_in;
        running.set(true);
        mbot_th_handle = std::move(std::thread(&mbot::recv_th)); // pass 'this' as the first argument
    }

    //tell the host theres a new mac address
    this->send_timesync();
}

// Destructor for mbot class. This function is not thread safe! Mbots should be not be freed concurrently.
mbot::~mbot()
{
    // Remove the robot from the swarm
    std::string mac_str = mac_to_string(mac_address);
    mbots.erase(mac_str);

    // Decrement the number of robots
    num_mbots.set(num_mbots.get() - 1);

    // Stop the thread if there are no more robots
    if (num_mbots.get() == 0)
    {
        running.set(false);
        send_th_handle.join();
        mbot_th_handle.join();
    }
}

// Converts uin8_t[MAC_ADDR_LEN] to a string where each 2 bytes are separated by semicolons
std::string mbot::mac_to_string(const mac_address_t mac_address)
{
    std::stringstream ss;
    for (int i = 0; i < 6; ++i)
    {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(mac_address[i]);
        if (i != 5) ss << ':';
    }
    return ss.str();
}

serial_twist2D_t mbot::get_robot_vel()
{
    return this->robot_vel.get();
}

serial_mbot_imu_t mbot::get_imu()
{
    return this->imu.get();
}

serial_mbot_motor_vel_t mbot::get_motor_vel()
{
    return this->motor_vel.get();
}

serial_twist2D_t mbot::get_robot_vel_goal()
{
    return this->robot_vel_goal;
}

serial_mbot_motor_vel_t mbot::get_motor_vel_goal()
{
    return this->motor_vel_goal;
}

void mbot::set_robot_vel_goal(float vx, float vy, float wz)
{
    this->robot_vel_goal.vx = vx;
    this->robot_vel_goal.vy = vy;
    this->robot_vel_goal.wz = wz;

    // Initialize variables for packet
    packet_t packet;
    size_t msg_len = sizeof(serial_twist2D_t);
    packet.length = msg_len + ROS_PKG_LEN + MAC_ADDR_LEN + 3;
    packet.data = new uint8_t[packet.length];
    uint8_t *msg_serialized = new uint8_t[msg_len];

    // Serialize message and create packet
    twist2D_t_serialize(&robot_vel_goal, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_VEL_CMD, this->mac_address, packet.data, packet.length);
    delete[] msg_serialized;

    // add to the send queue TODO: figure out how we specify MAC for host to send to
    this->send_mutex.lock();
    this->send_queue.push(packet);
    this->send_mutex.unlock();

    // alert the send thread there is work to do
    this->send_cv.notify_one();
}

void mbot::set_motor_vel_goal(float a, float b, float c = 0.0f)
{
    // TODO: make sure I'm doing this right
    this->motor_vel_goal.velocity[0] = a;
    this->motor_vel_goal.velocity[1] = b;
    this->motor_vel_goal.velocity[2] = c;

    // Initialize variables for packet
    packet_t packet;
    size_t msg_len = sizeof(serial_mbot_motor_vel_t);
    packet.length = msg_len + ROS_PKG_LEN + MAC_ADDR_LEN+ 3;
    packet.data = new uint8_t[packet.length];
    uint8_t *msg_serialized = new uint8_t[msg_len];

    // Serialize message and create packet
    mbot_motor_vel_t_serialize(&motor_vel_goal, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_MOTOR_VEL_CMD, this->mac_address, packet.data, packet.length);
    delete[] msg_serialized;

    // add to the send queue TODO: figure out how we specify MAC for host to send to
    this->send_mutex.lock();
    this->send_queue.push(packet);
    this->send_mutex.unlock();

    // alert the send thread there is work to do
    this->send_cv.notify_one();
}

serial_mbot_motor_pwm_t mbot::get_motor_pwm()
{
    return this->motor_pwm.get();
}

void mbot::set_motor_pwm(float a, float b, float c = 0.0f)
{
    serial_mbot_motor_pwm_t mbot_pwm;
    mbot_pwm.pwm[0] = a;
    mbot_pwm.pwm[1] = b;
    mbot_pwm.pwm[2] = c;
    this->motor_pwm.set(mbot_pwm);

    // Initialize variables for packet
    packet_t packet;
    size_t msg_len = sizeof(serial_mbot_motor_pwm_t);
    packet.length = msg_len + ROS_PKG_LEN + MAC_ADDR_LEN+ 3;
    packet.data = new uint8_t[packet.length];
    uint8_t *msg_serialized = new uint8_t[msg_len];

    // Serialize message and create packet
    mbot_motor_pwm_t_serialize(&mbot_pwm, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_MOTOR_PWM_CMD, this->mac_address, packet.data, packet.length);
    delete[] msg_serialized;

    // add to the send queue TODO: figure out how we specify MAC for host to send to
    this->send_mutex.lock();
    this->send_queue.push(packet);
    this->send_mutex.unlock();

    // alert the send thread there is work to do
    send_cv.notify_one();
}

serial_pose2D_t mbot::get_odom()
{
    return this->odom.get();
}

serial_mbot_encoders_t mbot::get_encoders()
{
    return this->encoders.get();
}

void mbot::set_odom(float x, float y, float theta)
{
    serial_pose2D_t mbot_odom;
    mbot_odom.x = x;
    mbot_odom.y = y;
    mbot_odom.theta = theta;
    this->odom.set(mbot_odom);

    // Initialize variables for packet
    packet_t packet;
    size_t msg_len = sizeof(serial_pose2D_t);
    packet.length = msg_len + ROS_PKG_LEN + MAC_ADDR_LEN+ 3;
    packet.data = new uint8_t[packet.length];
    uint8_t *msg_serialized = new uint8_t[msg_len];

    // Serialize message and create packet
    pose2D_t_serialize(&mbot_odom, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_ODOMETRY_RESET, this->mac_address, packet.data, packet.length);
    delete[] msg_serialized;

    // add to the send queue TODO: figure out how we specify MAC for host to send to
    this->send_mutex.lock();
    this->send_queue.push(packet);
    this->send_mutex.unlock();

    // alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::reset_odom()
{
    serial_pose2D_t mbot_odom;
    mbot_odom.x = 0;
    mbot_odom.y = 0;
    mbot_odom.theta = 0;
    this->odom.set(mbot_odom);

    // Initialize variables for packet
    packet_t packet;
    size_t msg_len = sizeof(serial_pose2D_t);
    packet.length = msg_len + ROS_PKG_LEN + MAC_ADDR_LEN+ 3;
    packet.data = new uint8_t[packet.length];
    uint8_t *msg_serialized = new uint8_t[msg_len];

    // Serialize message and create packet
    pose2D_t_serialize(&mbot_odom, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_ODOMETRY_RESET, this->mac_address, packet.data, packet.length);
    delete[] msg_serialized;

    // add to the send queue TODO: figure out how we specify MAC for host to send to
    this->send_mutex.lock();
    this->send_queue.push(packet);
    this->send_mutex.unlock();

    // alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::set_encoders(int a, int b, int c = 0)
{
    serial_mbot_encoders_t mbot_encoders;
    mbot_encoders.ticks[0] = a;
    mbot_encoders.ticks[1] = b;
    mbot_encoders.ticks[2] = c;
    this->encoders.set(mbot_encoders);

    // Initialize variables for packet
    packet_t packet;
    size_t msg_len = sizeof(serial_mbot_encoders_t);
    packet.length = msg_len + ROS_PKG_LEN + MAC_ADDR_LEN+ 3;
    packet.data = new uint8_t[packet.length];
    uint8_t *msg_serialized = new uint8_t[msg_len];

    // Serialize message and create packet
    mbot_encoders_t_serialize(&mbot_encoders, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_ENCODERS_RESET, this->mac_address, packet.data, packet.length);
    delete[] msg_serialized;

    // add to the send queue TODO: figure out how we specify MAC for host to send to
    this->send_mutex.lock();
    this->send_queue.push(packet);
    this->send_mutex.unlock();

    // alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::reset_encoders()
{
    serial_mbot_encoders_t mbot_encoders;
    mbot_encoders.ticks[0] = 0;
    mbot_encoders.ticks[1] = 0;
    mbot_encoders.ticks[2] = 0;
    this->encoders.set(mbot_encoders);

    // Initialize variables for packet
    packet_t packet;
    size_t msg_len = sizeof(serial_mbot_encoders_t);
    packet.length = msg_len + ROS_PKG_LEN + MAC_ADDR_LEN+ 3;
    packet.data = new uint8_t[packet.length];
    uint8_t *msg_serialized = new uint8_t[msg_len];

    // Serialize message and create packet
    mbot_encoders_t_serialize(&mbot_encoders, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_ENCODERS_RESET, this->mac_address, packet.data, packet.length);
    delete[] msg_serialized;

    // add to the send queue
    this->send_mutex.lock();
    this->send_queue.push(packet);
    this->send_mutex.unlock();

    // alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::send_timesync(){
    serial_timestamp_t timestamp;
    auto currentTimePoint = std::chrono::high_resolution_clock::now();
    
    // Get the time since epoch in microseconds
    auto microsecondsSinceEpoch = std::chrono::time_point_cast<std::chrono::microseconds>(currentTimePoint)
                                      .time_since_epoch()
                                      .count();
    timestamp.utime = microsecondsSinceEpoch;

    // Initialize variables for packet
    packet_t packet;
    size_t msg_len = sizeof(serial_timestamp_t);
    packet.length = msg_len + ROS_PKG_LEN + MAC_ADDR_LEN + 3;
    packet.data = new uint8_t[packet.length];
    uint8_t *msg_serialized = new uint8_t[msg_len];

    // Serialize message and create packet
    timestamp_t_serialize(&timestamp, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_TIMESYNC, this->mac_address, packet.data, packet.length);
    delete[] msg_serialized;

    // add to the send queue
    this->send_mutex.lock();
    std::cout << "adding to queue\n";
    this->send_queue.push(packet);
    this->send_mutex.unlock();

    // alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::update_mbot(message_topics topic, uint8_t *data)
{
    switch (topic)
    {
    case MBOT_ODOMETRY:
        this->odom.set(*((serial_pose2D_t *)data));
        break;
    case MBOT_IMU:
        this->imu.set(*((serial_mbot_imu_t *)data));
        break;
    case MBOT_ENCODERS:
        this->encoders.set(*((serial_mbot_encoders_t *)data));
        break;
    case MBOT_MOTOR_VEL:
        this->motor_vel.set(*((serial_mbot_motor_vel_t *)data));
        break;
    case MBOT_MOTOR_PWM:
        this->motor_pwm.set(*((serial_mbot_motor_pwm_t *)data));
        break;
    case MBOT_VEL:
        this->robot_vel.set(*((serial_twist2D_t *)data));
        break;
    default:
        break;
    }
}

// Reads all incoming data on USB
void mbot::recv_th()
{
    // open com port host is connected to
    // get host mac address and save it
    struct termios tty;
    port_name.replace(0, 3, "/dev/ttyS");
    serial_port = open(port_name.c_str(), O_RDWR);
    if (serial_port == -1)
    {
        perror("Error opening serial port");
        return;
    }

    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_port, &tty) != 0)
    {
        perror("Error from tcgetattr");
        return;
    }

    tty.c_cflag = B921600; // Set your desired baud rate
    tty.c_cflag |= CS8;    // 8-bit data
    tty.c_cflag |= CLOCAL; // Ignore modem control lines
    tty.c_cflag |= CREAD;  // Enable receiver

    tcflush(serial_port, TCIFLUSH);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        perror("Error from tcsetattr");
        return;
    }

    // start the write thread now that the serial port is open
    send_th_handle = std::thread(&mbot::send_th);
    while (running.get())
    {
        char buffer[256];
        // Read data from the device, should get ACK

        mac_address_t mac_address;
        uint8_t checksum_val;
        uint16_t pkt_len;
        read_mac_address(serial_port, mac_address, &pkt_len);

        uint8_t header_data[ROS_HEADER_LEN];
        read_header(serial_port, header_data);
        if (!validate_header(header_data))
            continue; // continue if header is invalid

        uint16_t message_len = ((uint16_t)header_data[3] << 8) + (uint16_t)header_data[2];
        uint16_t topic_id = ((uint16_t)header_data[6] << 8) + (uint16_t)header_data[5];
        uint8_t msg_data_serialized[message_len];
        char topic_msg_data_checksum = 0;
        read_message(serial_port, msg_data_serialized, message_len, &topic_msg_data_checksum);
        if (!validate_message(header_data, msg_data_serialized, message_len, topic_msg_data_checksum))
            continue;

        std::string mac_str = mac_to_string(mac_address);
        if (mbots.find(mac_str) == mbots.end())
            continue; // continue if robot is not in swarm
        mbot *curr_mbot = mbots[mac_str];

        curr_mbot->update_mbot((message_topics)topic_id, msg_data_serialized);
    }
}

void mbot::send_th()
{
    while (running.get())
    {
        packet_t packet;
        { // scope for which we need the lock on the queue
            std::unique_lock<std::mutex> queue_lock(send_mutex);
            while (send_queue.empty())
            {
                send_cv.wait(queue_lock);
            }
            packet = send_queue.front();
            send_queue.pop();
        }
        ssize_t bytes_written = write(serial_port, packet.data, packet.length);
        if (bytes_written < 0)
        {
            perror("Error writing to serial port");
        }
        else
        {
            printf("Sent %ld bytes\n", bytes_written);
            char buffer[256];
            // Read data from the device, should get ACK
            if (bytes_written == 25){
                int bytes_read = read(serial_port, buffer, sizeof(buffer));
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    printf("Received: %s", buffer);
                }
            }
        }
    }
}