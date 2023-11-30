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

// mbot member function definitions

// Static member variables
std::atomic<bool> mbot::running(false);
std::atomic<bool> mbot::server_running(false);
std::atomic<int> mbot::min_msg_rate(15);
std::atomic<bool> mbot::fast(false);
std::atomic<bool> mbot::verbose(false);
std::atomic<int> mbot::num_mbots;
std::atomic<uint64_t> mbot::start_time;

std::string mbot::port;
int mbot::serial_port;
bool mbot::initialized = false;

std::unordered_map<std::string, mbot *> mbot::mbots;
std::queue<mbot::packet_t> mbot::send_queue;

std::thread mbot::server_th_handle;
std::thread mbot::mbot_th_handle;
std::thread mbot::send_th_handle;

telemetry_server mbot::server;

std::function<void(mbot *)> mbot::update_cb;

std::mutex mbot::send_mutex;
std::mutex mbot::update_cb_mutex;
std::condition_variable mbot::send_cv;

// Contructor
mbot::mbot(const std::string &name, const std::string &mac)
{
    if (!mbot::initialized)
    {
        throw(std::runtime_error("mbot not initialized, please call mbot::init before instantiating mbot objects"));
    }

    this->name = name;
    this->alive = true;
    this->mac = mac;
    mac_string_to_bytes(mac, mac_bytes);

    // Add the robot to the swarm
    mbots.insert(std::pair<std::string, mbot *>(this->mac, this));

    // Increment the number of robots
    num_mbots++;

    // Start the thread
    if (!running.load() && num_mbots.load() == 1)
    {
        running.store(true);

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

// Copy constructor
mbot::mbot(const mbot &other)
{
    // Copy basic parameters
    this->name = other.name;
    this->mac = other.mac;
    std::memcpy(this->mac_bytes, other.mac_bytes, MAC_ADDR_LEN);
    this->alive.store(other.alive);

    // Update map with new pointer
    // Updating the map in this way means that only the most recent copy of an mbot
    // will be updated by the recv_th thread. I think this is the best (maybe only?) way
    // to handle this situation. If we used the original pointer, then it could be deleted
    mbots[mac] = this;
}

// Destructor
mbot::~mbot()
{
    // Remove the robot from the swarm
    // This check occurs because of the copy constructor.
    // Because they share the same MAC address, we only
    // want to remove the robot if it is the most recent copy.
    if (mbots[mac] == this)
    {
        mbots.erase(mac);
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

void mbot::init(const std::string &port)
{
    mbot::port = port;
    mbot::initialized = true;
}

// Getters
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

serial_mbot_motor_pwm_t mbot::get_motor_pwm()
{
    return this->motor_pwm.load();
}

serial_pose2D_t mbot::get_odom()
{
    return this->odom.load();
}

serial_mbot_encoders_t mbot::get_encoders()
{
    return this->encoders.load();
}

int mbot::get_msg_rate()
{
    return this->msg_rate.load();
}

bool mbot::is_alive()
{
    return alive.load();
}

// Setters
void mbot::set_robot_vel_goal(float vx, float vy, float wz)
{
    if (!alive || !running.load())
        return;
    this->robot_vel_goal.vx = vx;
    this->robot_vel_goal.vy = vy;
    this->robot_vel_goal.wz = wz;

    // Serialize and send the message
    encode_and_push_msg(&robot_vel_goal, MBOT_VEL_CMD, this->mac_bytes);
}


void mbot::set_motor_vel_goal(float a, float b, float c = 0.0f)
{
    if (!alive || !running.load())
        return;
    this->motor_vel_goal.velocity[0] = a;
    this->motor_vel_goal.velocity[1] = b;
    this->motor_vel_goal.velocity[2] = c;

    // Serialize message and create packet
    encode_and_push_msg(&motor_vel_goal, MBOT_MOTOR_VEL_CMD, this->mac_bytes);
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
    encode_and_push_msg(&mbot_pwm, MBOT_MOTOR_PWM_CMD, this->mac_bytes);
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
    encode_and_push_msg(&mbot_odom, MBOT_ODOMETRY_RESET, this->mac_bytes);
}

void mbot::reset_odom()
{
    set_odom(0, 0, 0);
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
    encode_and_push_msg(&mbot_encoders, MBOT_ENCODERS_RESET, this->mac_bytes);
}

void mbot::reset_encoders()
{
    set_encoders(0, 0, 0);
}

void mbot::send_timesync()
{
    if (!alive || !running.load())
        return;
    serial_timestamp_t timestamp;

    timestamp.utime = get_time_us() - start_time.load();

    // Serialize message and create packet
    encode_and_push_msg(&timestamp, MBOT_TIMESYNC, this->mac_bytes);
}

// Public static functions
void mbot::set_verbose(bool state)
{
    verbose.store(state);
}

void mbot::set_min_msg_rate(int rate)
{
    min_msg_rate.store(rate);
}

void mbot::set_fast(bool state) 
{
    fast.store(state);
}

bool mbot::is_running()
{
    return running.load();
}

void mbot::start_server(uint16_t port)
{
    server_running.store(true);
    server_th_handle = std::move(std::thread(&telemetry_server::run,
                                             &server,
                                             port));
}

void mbot::set_on_update(std::function<void(mbot *)> callback)
{
    std::lock_guard<std::mutex> lock(update_cb_mutex);
    update_cb = callback;
}

// packet_t member function definitions

// Default constructor
mbot::packet_t::packet_t()
{
    data = nullptr;
    len = 0;
}

// Constructor with length
mbot::packet_t::packet_t(size_t len)
{
    data = new uint8_t[len];
    this->len = len;
}

// Copy constructor
mbot::packet_t::packet_t(const mbot::packet_t &other)
{
    this->data = new uint8_t[other.len];
    std::memcpy(this->data, other.data, other.len);
    this->len = other.len;
}

// Destructor
mbot::packet_t::~packet_t()
{
    if (data != nullptr)
    {
        delete[] data;
    }
}

// Assignment operator
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

// mbot private member function definitions

// Virtual functions
void mbot::on_update()
{
    // Do nothing
}

serial_pose2D_t mbot::get_functional_pose()
{
    return get_odom();
}


void mbot::update_mbot(packets_wrapper_t *pkt)
{
    this->encoders.store(pkt->encoders);
    this->odom.store(pkt->odom);
    this->imu.store(pkt->imu);
    this->robot_vel.store(pkt->mbot_vel);
    this->motor_vel.store(pkt->motor_vel);
    this->motor_pwm.store(pkt->motor_pwm);

    // Call the virtual on_update function
    this->on_update();

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

uint64_t mbot::get_time_us()
{
    auto currentTimePoint = std::chrono::high_resolution_clock::now();
    auto microsecondsSinceEpoch = std::chrono::time_point_cast<std::chrono::microseconds>(currentTimePoint);
    return (uint64_t)microsecondsSinceEpoch.time_since_epoch().count();
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
        perror("Error: Unable to open serial port.\n");
        return;
    }

    if (tcgetattr(serial_port, &tty) != 0)
    {
        perror("Error: Unable to get serial port attributes.\n");
        return;
    }

    cfsetospeed(&tty, B921600); // Set output baud rate
    cfsetispeed(&tty, B921600); // Set input baud rate

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_cflag |= (CLOCAL | CREAD);        // Ignore modem control lines, enable receiver
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
    
    std::ofstream log;
    if (verbose.load())
    {
        log = std::ofstream("log.txt", std::ios::out);
        if (!log.is_open())
        {
            std::cerr << "Error opening the log file!" << std::endl;
            verbose.store(false);
        }
    }

    std::unordered_map<std::string, int> msg_counts;
    uint64_t curr_time = get_time_us();
    while (running.load())
    {
        bool is_fast = fast.load();
        bool timeout = false;
        size_t timeout_count = 0;
        uint8_t trigger_val = 0x00;
        std::vector<char> buffer; // Need to use vector becuase of null terminator in data

        // Wait for the trigger byte, append all other valid ascii bytes to buffer
        while (trigger_val != 0xff)
        {
            read(serial_port, &trigger_val, 1);
            if (is_fast)
                continue;

            // Only append valid ascii characters
            timeout_count++;
            if (trigger_val > 0 && trigger_val < 128)
                buffer.push_back(trigger_val);

            if (timeout_count > 512)
            {
                timeout = true;
                break;
            }
        }

        if (!is_fast) {
            // Convert buffer to string, log if verbose
            std::string buf_str(buffer.begin(), buffer.end());
            if (verbose.load())
                log << buf_str << std::flush;

            // Check if host crashed
            if (buf_str.find("bootloader") != std::string::npos)
            {
                std::cerr << "Error: host crashed. Attempting to reconnect ..." << std::endl;
                reconnect();
                continue;
            }
            // Check if timeout
            if (timeout)
                continue;
        }

        // Read MAC address and packet length
        mac_address_t mac;
        uint8_t checksum_val;
        uint16_t pkt_len;
        read_mac_address(mac, &pkt_len);
        if (pkt_len != 204)
            continue;

        // Read message and checksum
        uint8_t msg_data_serialized[pkt_len];
        uint8_t data_checksum = 0;
        read_message(msg_data_serialized, pkt_len, &data_checksum);

        // Validate message
        if (!validate_message(msg_data_serialized, pkt_len, data_checksum))
            continue;

        

        // Update the robot
        std::string mac_str = mac_bytes_to_string(mac);
        if (mbots.find(mac_str) == mbots.end())
            continue;
        mbot *curr_mbot = mbots[mac_str];
        packets_wrapper_t *pkt_wrapped = (packets_wrapper_t *)msg_data_serialized;
        curr_mbot->update_mbot(pkt_wrapped);



        // If server is running, publish functional pose
        if (server_running.load())
        {
            serial_pose2D_t odom = curr_mbot->get_functional_pose();
            std::cout << "publishing odom x: " << odom.x << "\n";
            server.send_data(jsonify_data(curr_mbot->mac, odom));
        }

        if (is_fast)
            continue;

        // Update message counts
        if (msg_counts.find(mac_str) == msg_counts.end())
            msg_counts[mac_str] = 0;
        msg_counts[mac_str]++;

        // Check if robot is alive
        if (curr_time + 1000000 < get_time_us())
        {
            int min_rate = mbot::min_msg_rate.load();
            for (auto &msg_count : msg_counts)
            {
                // Get the robot pointer and message rate
                mbot *mbot_ptr = mbots[msg_count.first];
                int msg_rate = msg_count.second;

                // Get the robot name and set the alive state
                std::string name = mbot_ptr->name;
                bool state = true;

                // Check if the robot is alive
                if (msg_count.second == 0)
                {
                    std::cerr << "Error: " << name << " not sending messages.\n";
                    state = false;
                }
                // Check if the message rate is too low
                else if (msg_rate < min_rate)
                    std::cerr << "Warning: " << name << " sending at " << msg_rate << " HZ (min rate: " << min_rate << " Hz)\n";

                // Reset the message count and set the alive state
                msg_count.second = 0;
                mbot_ptr->alive.store(state);
                mbot_ptr->msg_rate.store(msg_rate);
            }
            curr_time = get_time_us();
        }
    }
    log.close();
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

// Helper functions for encoding and decoding messages
std::string mbot::jsonify_data(std::string mac, serial_pose2D_t odom)
{
    std::ostringstream oss;
    oss << "{"
        << "\"mac\":\"" << mac << "\","
        << "\"x\":" << odom.x << ","
        << "\"y\":" << odom.y << ","
        << "\"theta\":" << odom.theta
        << "}";

    return oss.str();
}

// MAC address in bytes to string
std::string mbot::mac_bytes_to_string(const mac_address_t mac)
{
    std::stringstream ss;
    for (int i = 0; i < 6; ++i)
    {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(mac[i]);
        if (i != 5)
            ss << ':';
    }
    return ss.str();
}

// MAC address in string to bytes
void mbot::mac_string_to_bytes(const std::string &mac_str, mac_address_t mac)
{
    std::stringstream ss(mac_str);
    int i = 0;
    while (ss.good() && i < 6)
    {
        std::string substr;
        getline(ss, substr, ':');
        mac[i] = std::stoi(substr, nullptr, 16);
        i++;
    }
}

void mbot::read_bytes(uint8_t *buffer, uint16_t len)
{
    size_t bytes_read = 0;
    while (bytes_read < len)
    {
        bytes_read += read(serial_port, buffer + bytes_read, len - bytes_read);
    }
}

void mbot::read_mac_address(uint8_t *mac, uint16_t *pkt_len)
{
    uint8_t pkt_len_buf[2];
    read_bytes(pkt_len_buf, 2);
    *pkt_len = ((uint16_t)pkt_len_buf[1] << 8) | (uint16_t)pkt_len_buf[0];
    read_bytes(mac, MAC_ADDR_LEN);
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

template <typename T>
void mbot::encode_and_push_msg(T *msg, uint16_t topic, mac_address_t mac)
{
    size_t msg_len = sizeof(T);
    packet_t pkt(msg_len + ROS_PKG_LEN + MAC_ADDR_LEN + 3);

    // add mac address
    pkt.data[0] = SYNC_FLAG;
    pkt.data[1] = (uint8_t)((msg_len + ROS_PKG_LEN) % 255);
    pkt.data[2] = (uint8_t)((msg_len + ROS_PKG_LEN) >> 8);
    std::memcpy(pkt.data + 3, mac, MAC_ADDR_LEN);

    // add ROS packet header
    pkt.data[9] = SYNC_FLAG;
    pkt.data[10] = VERSION_FLAG;
    pkt.data[11] = (uint8_t)(msg_len % 255);
    pkt.data[12] = (uint8_t)(msg_len >> 8);
    uint8_t cs1_addends[2] = {pkt.data[11], pkt.data[12]};
    pkt.data[13] = checksum(cs1_addends, 2);

    // add topic and message
    pkt.data[14] = (uint8_t)(topic % 255);
    pkt.data[15] = (uint8_t)(topic >> 8);
    std::memcpy(pkt.data + 16, msg, msg_len);
    uint8_t cs2_addends[msg_len + 2];
    cs2_addends[0] = pkt.data[14];
    cs2_addends[1] = pkt.data[15];
    std::memcpy(cs2_addends + 2, (uint8_t *)msg, msg_len);
    pkt.data[16 + msg_len] = checksum(cs2_addends, msg_len + 2);

    // add to the send queue
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        send_queue.push(pkt);
    }

    // alert the send thread there is work to do
    send_cv.notify_one();
}

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

std::vector<std::string> get_macs_from_file(const std::string &file_name)
{
    std::vector<std::string> macs;
    std::ifstream file(file_name);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << file_name << std::endl;
        return macs;
    }

    std::string line;
    while (std::getline(file, line))
    {
        macs.push_back(line);
    }
    file.close();
    return macs;
}