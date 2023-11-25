#include <cstring>
#include <type_traits>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <dirent.h>
#include <string.h>
#include <fstream>

#include "mbot.h"

#ifndef B921600
#define B921600 921600
#endif

/* packet defintiion */

mbot::packet_t::packet_t()
{
    data = nullptr;
    len = 0;
}

mbot::packet_t::packet_t(size_t len)
{
    data = new uint8_t[len];
    this->len = len;
}

mbot::packet_t::packet_t(const mbot::packet_t &other)
{
    this->data = new uint8_t[other.len];
    std::memcpy(this->data, other.data, other.len);
    this->len = other.len;
}

mbot::packet_t::~packet_t()
{
    if (data != nullptr)
    {
        delete[] data;
    }
}

mbot::packet_t &mbot::packet_t::operator=(const mbot::packet_t &other)
{
    if (this != &other)
    {
        this->data = new uint8_t[other.len];
        std::memcpy(this->data, other.data, other.len);
        this->len = other.len;
    }
    return *this;
}

/* mbot member function definitions */

// Set static bool false for initial constructor
std::atomic<bool> mbot::running(false);
std::unordered_map<std::string, mbot *> mbot::mbots;
std::atomic<int> mbot::num_mbots;
std::thread mbot::mbot_th_handle;
int mbot::serial_port;
std::string mbot::port;
std::atomic<uint64_t> mbot::start_time;

std::atomic<bool> mbot::server_running(false);
telemetry_server mbot::server;
std::thread mbot::server_th_handle;

std::thread mbot::send_th_handle;
std::mutex mbot::send_mutex;
std::queue<mbot::packet_t> mbot::send_queue;
std::condition_variable mbot::send_cv;
std::atomic<bool> mbot::verbose;
std::atomic<int> mbot::min_msg_rate(15);
std::function<void(mbot*)> mbot::update_cb;
std::mutex mbot::update_cb_mutex;

// Constructor for mbot class. This function is not thread safe! Mbots should not be instantiated concurrently.
mbot::mbot(const std::string &name, const std::string &mac_address)
{
    this->name = name;
    this->alive = true;
    string_to_mac(mac_address, this->mac_address);

    // Add the robot to the swarm
    mbots.insert(std::pair<std::string, mbot *>(mac_address, this));

    // Increment the number of robots
    num_mbots++;

    // Start the thread
    if (!running.load() && num_mbots.load() == 1)
    {
        running.store(true);
        verbose.store(false);

        // Set start time for timesync offset
        start_time.store(get_time_us());

        // Init serial port
        init_serial();

        // Start the threads
        mbot_th_handle = std::move(std::thread(&mbot::recv_th));
        send_th_handle = std::move(std::thread(&mbot::send_th));
    }

    // Send timesync to pair client with host
    this->send_timesync();
}

// Destructor for mbot class. This function is not thread safe! Mbots should be not be freed concurrently.
mbot::~mbot()
{
    // Remove the robot from the swarm
    // This check occurs because of the copy constructor.
    // Because they share the same MAC address, we only
    // want to remove the robot if it is the most recent copy.
    std::string mac_str = mac_to_string(mac_address);
    if (mbots[mac_str] == this)
    {
        mbots.erase(mac_str);
    }

    // Decrement the number of robots
    num_mbots--;

    // Stop the thread if there are no more robots
    if (num_mbots.load() == 0)
    {
        running.store(false);
        send_cv.notify_one();
        send_th_handle.join();
        mbot_th_handle.join();
    }
}

// Copy constructor for mbot class
mbot::mbot(const mbot &other)
{
    // Copy basic parameters
    this->name = other.name;
    std::memcpy(this->mac_address, other.mac_address, MAC_ADDR_LEN);
    this->alive.store(other.alive);

    // Update map with new pointer
    // Updating the map in this way means that only the most recent copy of an mbot
    // will be updated by the recv_th thread. I think this is the best (maybe only?) way
    // to handle this situation. If we used the original pointer, then it could be deleted
    std::string mac_str = mac_to_string(mac_address);
    mbots[mac_str] = this;
}

std::vector<mbot> mbot::init_from_file(const std::string &file_name)
{
    // Given a file with a list of MAC addresses, return an array of mbots
    std::ifstream file(file_name);
    if (!file.is_open())
    {
        perror("Error opening MAC address file. Ensure specified path is correct.\n");
        return std::vector<mbot>();
    }
    std::string line;

    // Read the file and get the number of bots and the mac addresses
    int num_bots = 0;
    std::vector<std::string> macs;
    while (std::getline(file, line))
    {
        if (line[0] == '#')
            continue;
        macs.push_back(line);
        num_bots++;
    }

    // Create the mbots
    std::vector<mbot> mbots;
    mbots.reserve(num_bots);
    for (int i = 0; i < num_bots; i++)
    {
        std::string name = "mbot" + std::to_string(i);
        mbots.emplace_back(name, macs[i]);
    }

    return mbots;
}

// Converts uin8_t[MAC_ADDR_LEN] to a string where each 2 bytes are separated by semicolons
std::string mbot::mac_to_string(const mac_address_t mac_address)
{
    std::stringstream ss;
    for (int i = 0; i < 6; ++i)
    {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(mac_address[i]);
        if (i != 5)
            ss << ':';
    }
    return ss.str();
}

// Converts a string where each 2 bytes are separated by semicolons to uint8_t[MAC_ADDR_LEN]
void mbot::string_to_mac(const std::string &mac_str, mac_address_t mac_address)
{
    mac_address_t mac;
    std::stringstream ss(mac_str);
    int i = 0;
    while (ss.good() && i < 6)
    {
        std::string substr;
        getline(ss, substr, ':');
        mac_address[i] = std::stoi(substr, nullptr, 16);
        i++;
    }
}

serial_twist2D_t mbot::get_robot_vel()
{
    return this->robot_vel.load();
}

serial_mbot_imu_t mbot::get_imu()
{
    return this->imu.load();
}

serial_mbot_motor_vel_t mbot::get_motor_vel()
{
    return this->motor_vel.load();
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
    if (!alive || !running.load())
        return;
    this->robot_vel_goal.vx = vx;
    this->robot_vel_goal.vy = vy;
    this->robot_vel_goal.wz = wz;

    // Serialize message and create packet
    packet_t packet;
    encode_msg(&robot_vel_goal, MBOT_VEL_CMD, this->mac_address, &packet);

    // add to the send queue
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        this->send_queue.push(packet);
    }

    // alert the send thread there is work to do
    this->send_cv.notify_one();
}

void mbot::set_motor_vel_goal(float a, float b, float c = 0.0f)
{
    if (!alive || !running.load())
        return;
    this->motor_vel_goal.velocity[0] = a;
    this->motor_vel_goal.velocity[1] = b;
    this->motor_vel_goal.velocity[2] = c;

    // Serialize message and create packet
    packet_t packet;
    encode_msg(&motor_vel_goal, MBOT_MOTOR_VEL_CMD, this->mac_address, &packet);

    // add to the send queue
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        this->send_queue.push(packet);
    }

    // alert the send thread there is work to do
    this->send_cv.notify_one();
}

serial_mbot_motor_pwm_t mbot::get_motor_pwm()
{
    return this->motor_pwm.load();
}

void mbot::set_motor_pwm(float a, float b, float c = 0.0f)
{
    if (!alive || !running.load())
        return;
    serial_mbot_motor_pwm_t mbot_pwm;
    mbot_pwm.pwm[0] = a;
    mbot_pwm.pwm[1] = b;
    mbot_pwm.pwm[2] = c;
    this->motor_pwm.store(mbot_pwm);

    // Serialize message and create packet
    packet_t packet;
    encode_msg(&mbot_pwm, MBOT_MOTOR_PWM_CMD, this->mac_address, &packet);

    // add to the send queue
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        this->send_queue.push(packet);
    }

    // alert the send thread there is work to do
    send_cv.notify_one();
}

serial_pose2D_t mbot::get_odom()
{
    return this->odom.load();
}

serial_mbot_encoders_t mbot::get_encoders()
{
    return this->encoders.load();
}

void mbot::set_odom(float x, float y, float theta)
{
    if (!alive || !running.load())
        return;
    serial_pose2D_t mbot_odom;
    mbot_odom.x = x;
    mbot_odom.y = y;
    mbot_odom.theta = theta;

    // Serialize message and create packet
    packet_t packet;
    encode_msg(&mbot_odom, MBOT_ODOMETRY_RESET, this->mac_address, &packet);

    // add to the send queue
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        this->send_queue.push(packet);
    }

    // alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::reset_odom()
{
    if (!alive || !running.load())
        return;
    serial_pose2D_t mbot_odom;
    mbot_odom.x = 0;
    mbot_odom.y = 0;
    mbot_odom.theta = 0;

    // Serialize message and create packet
    packet_t packet;
    encode_msg(&mbot_odom, MBOT_ODOMETRY_RESET, this->mac_address, &packet);

    // add to the send queue
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        this->send_queue.push(packet);
    }

    // alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::set_encoders(int a, int b, int c = 0)
{
    if (!alive || !running.load())
        return;
    serial_mbot_encoders_t mbot_encoders;
    mbot_encoders.ticks[0] = a;
    mbot_encoders.ticks[1] = b;
    mbot_encoders.ticks[2] = c;

    // Serialize message and create packet
    packet_t packet;
    encode_msg(&mbot_encoders, MBOT_ENCODERS_RESET, this->mac_address, &packet);

    // add to the send queue
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        this->send_queue.push(packet);
    }

    // alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::reset_encoders()
{
    if (!alive || !running.load())
        return;
    serial_mbot_encoders_t mbot_encoders;
    mbot_encoders.ticks[0] = 0;
    mbot_encoders.ticks[1] = 0;
    mbot_encoders.ticks[2] = 0;

    // Serialize message and create packet
    packet_t packet;
    encode_msg(&mbot_encoders, MBOT_ENCODERS_RESET, this->mac_address, &packet);

    // add to the send queue
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        this->send_queue.push(packet);
    }

    // alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::send_timesync()
{
    if (!alive || !running.load())
        return;
    serial_timestamp_t timestamp;

    timestamp.utime = get_time_us() - start_time.load();

    // Serialize message and create packet
    packet_t packet;
    encode_msg(&timestamp, MBOT_TIMESYNC, this->mac_address, &packet);

    // add to the send queue
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        this->send_queue.push(packet);
    }

    // alert the send thread there is work to do
    send_cv.notify_one();
}

void mbot::set_verbose(bool state)
{
    verbose.store(state);
}

void mbot::set_min_msg_rate(int rate)
{
    min_msg_rate.store(rate);
}

bool mbot::is_running()
{
    return running.load();
}

bool mbot::is_alive()
{
    return alive.load();
}

void mbot::start_server(uint16_t port)
{
    server_running.store(true);
    server_th_handle = std::move(std::thread(&telemetry_server::run,
                                             &server,
                                             port));
}

void mbot::on_update(std::function<void(mbot *)> callback)
{
    std::lock_guard<std::mutex> lock(update_cb_mutex);
    update_cb = callback;
}

void mbot::update_mbot(packets_wrapper_t *pkt)
{
    this->encoders.store(pkt->encoders);
    this->odom.store(pkt->odom);
    this->imu.store(pkt->imu);
    this->robot_vel.store(pkt->mbot_vel);
    this->motor_vel.store(pkt->motor_vel);
    this->motor_pwm.store(pkt->motor_pwm);

    // Call the user defined callback function
    std::lock_guard<std::mutex> lock(update_cb_mutex);
    if (update_cb != nullptr)
    {
        update_cb(this);
    }
}

void mbot::reconnect()
{
    for (auto &mbot : mbots)
    {
        mbot.second->send_timesync();
    }
}

// comms.h functions

uint8_t mbot::checksum(uint8_t *addends, int len)
{
    // takes in an array and sums the contents then checksums the array
    int sum = 0;
    for (int i = 0; i < len; i++)
    {
        sum += addends[i];
    }
    return 255 - ((sum) % 256);
}

void mbot::read_bytes(uint8_t *buffer, uint16_t len)
{
    size_t bytes_read = 0;
    while (bytes_read < len)
    {
        bytes_read += read(serial_port, buffer + bytes_read, len - bytes_read);
    }
}

void mbot::read_mac_address(uint8_t *mac_address, uint16_t *pkt_len)
{
    uint8_t pkt_len_buf[2];
    read_bytes(pkt_len_buf, 2);
    *pkt_len = ((uint16_t)pkt_len_buf[1] << 8) | (uint16_t)pkt_len_buf[0];
    read_bytes(mac_address, MAC_ADDR_LEN);
}

void mbot::read_message(uint8_t *data_serialized, uint16_t message_len, uint8_t *data_checksum)
{
    read_bytes(data_serialized, message_len);
    read_bytes(data_checksum, 1);
}

int mbot::validate_message(uint8_t *data_serialized, uint16_t message_len, uint8_t data_checksum)
{
    uint8_t cs_data = checksum(data_serialized, message_len);
    int valid_message = (cs_data == data_checksum);
    return valid_message;
}

template<typename T>
void mbot::encode_msg(T *msg, uint16_t topic, mac_address_t mac_address, packet_t *pkt)
{
    size_t msg_len = sizeof(T);
    *pkt = packet_t(msg_len + ROS_PKG_LEN + MAC_ADDR_LEN + 3);

    // add mac address
    pkt->data[0] = SYNC_FLAG;
    pkt->data[1] = (uint8_t)((msg_len + ROS_PKG_LEN) % 255);
    pkt->data[2] = (uint8_t)((msg_len + ROS_PKG_LEN) >> 8);
    std::memcpy(pkt->data + 3, mac_address, MAC_ADDR_LEN);

    // add ROS packet header
    pkt->data[9] = SYNC_FLAG;
    pkt->data[10] = VERSION_FLAG;
    pkt->data[11] = (uint8_t)(msg_len % 255);
    pkt->data[12] = (uint8_t)(msg_len >> 8);
    uint8_t cs1_addends[2] = {pkt->data[11], pkt->data[12]};
    pkt->data[13] = checksum(cs1_addends, 2);

    // add topic and message
    pkt->data[14] = (uint8_t)(topic % 255);
    pkt->data[15] = (uint8_t)(topic >> 8);
    std::memcpy(pkt->data + 16, msg, msg_len);
    uint8_t cs2_addends[msg_len + 2];
    cs2_addends[0] = pkt->data[14];
    cs2_addends[1] = pkt->data[15];
    std::memcpy(cs2_addends + 2, (uint8_t*)msg, msg_len);
    pkt->data[16 + msg_len] = checksum(cs2_addends, msg_len + 2);
}

uint64_t mbot::get_time_us()
{
    auto currentTimePoint = std::chrono::high_resolution_clock::now();
    auto microsecondsSinceEpoch = std::chrono::time_point_cast<std::chrono::microseconds>(currentTimePoint);
    return (uint64_t)microsecondsSinceEpoch.time_since_epoch().count();
}

std::string mbot::jsonify_packets_wrapper(mac_address_t mac_address, mbot::packets_wrapper_t *packets_wrapper)
{
    std::ostringstream oss;

    oss << "{"
        << "\"mac\":\"" << mac_to_string(mac_address) << "\","
        << "\"x\":" << packets_wrapper->odom.x << ","
        << "\"y\":" << packets_wrapper->odom.y << ","
        << "\"theta\":" << packets_wrapper->odom.theta << ","
        << "\"vx\":" << packets_wrapper->mbot_vel.vx << ","
        // << "\"vy\":" << packets_wrapper->mbot_vel.vy << ","
        << "\"wz\":" << packets_wrapper->mbot_vel.wz
        // << "\"a\":" << packets_wrapper->encoders.ticks[0] << ","
        // << "\"b\":" << packets_wrapper->encoders.ticks[1] << ","
        // << "\"c\":" << packets_wrapper->encoders.ticks[2]
        << "}";

    // Return the JSON string
    return oss.str();
}

void mbot::init_serial() 
{
    // Check if user defined port
    if (port.empty())
    {
        // But for now print an error
        perror("Error: No port specified.\n");
        return;
    }

    // open com port host is connected to
    struct termios tty;
    serial_port = open(port.c_str(), O_RDWR);
    if (serial_port == -1)
    {
        perror("Error opening serial port.\n");
        return;
    }

    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_port, &tty) != 0)
    {
        perror("Error from tcgetattr.");
        return;
    }

    cfsetospeed(&tty, B921600); // Set output baud rate
    cfsetispeed(&tty, B921600); // Set input baud rate

    tty.c_cflag |= (CLOCAL | CREAD);        // Ignore modem control lines, enable receiver
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_cflag &= ~CSIZE;                  // Clear the size bits
    tty.c_cflag |= CS8;                     // 8-bit data
    tty.c_cflag &= ~PARENB;                 // No parity
    tty.c_cflag &= ~CSTOPB;                 // 1 stop bit

    tcflush(serial_port, TCIFLUSH);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        perror("Error from tcsetattr");
        return;
    }
}

// Reads all incoming data on USB
void mbot::recv_th()
{   
    std::ofstream outputFile;
    if (verbose.load())
    {
        outputFile = std::ofstream("log.txt", std::ios::out);
        if (!outputFile.is_open())
        {
            std::cerr << "Error opening the log file!" << std::endl;
            verbose.store(false);
        }
    }

    std::unordered_map<std::string, int> msg_counts;
    uint64_t curr_time = get_time_us();
    while (running.load())
    {
        mac_address_t mac_address;
        uint8_t checksum_val;
        uint16_t pkt_len;

        bool timeout = false;
        size_t timeout_count = 0;
        uint8_t trigger_val = 0x00;
        std::vector<char> buffer; // Need to use vector becuase of null terminator in data
        while (trigger_val != 0xff)
        {
            read(serial_port, &trigger_val, 1);
            timeout_count++;

            // Only append valid ascii characters
            if (trigger_val > 0 && trigger_val < 128)
                buffer.push_back(trigger_val);

            if (timeout_count > 512)
            {
                timeout = true;
                break;
            }
        }

        std::string buf_str(buffer.begin(), buffer.end());
        if (verbose.load()) 
            outputFile << buf_str << std::flush;

        if (buf_str.find("boot:") != std::string::npos)
        {
            std::cerr << "Error: host crashed. Attempting to reconnect ..." << std::endl;
            reconnect();
        }

        if (timeout) continue;

        read_mac_address(mac_address, &pkt_len);
        if (pkt_len != 204) continue;

        uint8_t msg_data_serialized[pkt_len];
        uint8_t data_checksum = 0;
        read_message(msg_data_serialized, pkt_len, &data_checksum);

        if (!validate_message(msg_data_serialized, pkt_len, data_checksum)) continue;

        std::string mac_str = mac_to_string(mac_address);
        if (mbots.find(mac_str) == mbots.end()) continue;
        mbot *curr_mbot = mbots[mac_str];

        packets_wrapper_t *pkt_wrapped = (packets_wrapper_t *)msg_data_serialized;
        curr_mbot->update_mbot(pkt_wrapped);

        if (msg_counts.find(mac_str) == msg_counts.end())
            msg_counts[mac_str] = 0;
        msg_counts[mac_str]++;

        if (server_running.load())
            server.send_data(jsonify_packets_wrapper(mac_address, pkt_wrapped));

        if (curr_time + 1000000 < get_time_us())
        {
            curr_time = get_time_us();
            int min_rate = mbot::min_msg_rate.load();
            for (auto &msg_count : msg_counts)
            {
                if (msg_count.second < min_rate)
                    std::cerr << "Warning: MBot #" << msg_count.first << " sending at <" << min_rate << " Hz" << std::endl;
                msg_count.second = 0;
            }
        }
    }
    outputFile.close();
    close(serial_port);
}

void mbot::send_th()
{
    while (running.load())
    {
        packet_t packet;
        {
            std::unique_lock<std::mutex> queue_lock(send_mutex);
            while (send_queue.empty())
            {
                send_cv.wait(queue_lock);
                if (!running.load())
                    return;
            }
            packet = send_queue.front();
            send_queue.pop();
        }

        ssize_t bytes_written = write(serial_port, packet.data, packet.len);
        if (bytes_written < 0)
        {
            perror("Error writing to serial port");
            continue;
        }
    }
}