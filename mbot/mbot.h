#ifndef MBOT_H
#define MBOT_H

#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <unordered_map>
#include <string>
#include <queue>

#include <iomanip>
#include <sstream>

#include "msgtypes.h"
#include "mbot_params.h"
#include "comms.h"
#include "telemetry_server.h"

#ifndef B921600
#define B921600 921600
#endif

using mac_address_t = uint8_t[MAC_ADDR_LEN];

class mbot
{
public:
    // Constructors and destructors
    mbot(const std::string &name, const std::string &mac);
    mbot(const mbot &);
    ~mbot();
    static void init(const std::string &port);

    // Public member variables
    std::string mac;
    std::string name;

    // Getters
    serial_twist2D_t get_robot_vel();
    serial_mbot_imu_t get_imu();
    serial_mbot_motor_vel_t get_motor_vel();
    serial_twist2D_t get_robot_vel_goal();
    serial_mbot_motor_vel_t get_motor_vel_goal();
    serial_mbot_motor_pwm_t get_motor_pwm();
    serial_pose2D_t get_odom();
    serial_mbot_encoders_t get_encoders();
    int get_msg_rate();
    bool is_alive();

    // Setters
    void set_robot_vel_goal(float vx, float vy, float wz);
    void set_motor_vel_goal(float a, float b, float c);
    void set_motor_pwm(float a, float b, float c);
    void set_odom(float x, float y, float theta);
    void reset_odom();
    void set_encoders(int a, int b, int c);
    void reset_encoders();
    void send_timesync();

    // Static functions
    static int get_invalid_msg_count();
    static void reset_invalid_msg_count();
    static void set_verbose(bool state);
    static void set_min_msg_rate(int rate);
    static void set_fast(bool state);
    static bool is_running();
    static void start_server(uint16_t port = 9002);
    static void set_on_update(std::function<void(mbot *)> callback);
    static uint64_t get_time_us();

private:
    // Wrapper packet for serial data
    struct __attribute__((__packed__)) packets_wrapper_t
    {
        serial_mbot_encoders_t encoders;
        serial_pose2D_t odom;
        serial_mbot_imu_t imu;
        serial_twist2D_t mbot_vel;
        serial_mbot_motor_vel_t motor_vel;
        serial_mbot_motor_pwm_t motor_pwm;
    };

    // Packet object
    class packet_t
    {
    public:
        packet_t();
        packet_t(size_t len);
        packet_t(const packet_t &);
        ~packet_t();
        packet_t &operator=(const packet_t &);
        uint8_t *data;
        uint8_t len;
    };

    // Atomic variables for robot state
    std::atomic<serial_twist2D_t> robot_vel;
    std::atomic<serial_mbot_imu_t> imu;
    std::atomic<serial_mbot_motor_vel_t> motor_vel;
    std::atomic<serial_mbot_motor_pwm_t> motor_pwm;
    std::atomic<serial_pose2D_t> odom;
    std::atomic<serial_mbot_encoders_t> encoders;

    // Goal state for the robot
    serial_twist2D_t robot_vel_goal;
    serial_mbot_motor_vel_t motor_vel_goal;

    // Atomic flag to indicate if the robot is alive
    std::atomic<bool> alive;
    std::atomic<int> msg_rate;
    static std::atomic<int> num_invalid_msgs;
    static bool initialized;

    // User defined callback function for update
    static std::function<void(mbot *)> update_cb;
    virtual void on_update();
    virtual serial_pose2D_t get_functional_pose();

    // Threads
    static std::thread send_th_handle;
    static std::thread mbot_th_handle;
    static std::thread server_th_handle;

    // Mutexes and condition variables
    static std::mutex update_cb_mutex;
    static std::mutex send_mutex;
    static std::condition_variable send_cv;

    // Queue for packets to be sent
    static std::queue<packet_t> send_queue;

    // Server related variables
    static telemetry_server server;
    static std::atomic<bool> server_running;

    // Time related variables and functions
    static std::atomic<uint64_t> start_time;

    // Serial port
    static void init_serial();
    static std::string port;
    static int serial_port;

    // MAC address in bytes
    mac_address_t mac_bytes;

    // Functions for updating robot state
    void update_mbot(packets_wrapper_t *pkt);
    static void reconnect();

    // Static variables for thread states
    static std::atomic<bool> verbose;
    static std::atomic<int> num_mbots;
    static std::atomic<bool> running;
    static std::atomic<int> min_msg_rate;
    static std::atomic<bool> fast;

    // Map to store all instatiated mbot objects
    static std::unordered_map<std::string, mbot *> mbots;

    // Thread functions for receiving and sending data
    static void recv_th();
    static void send_th();

    // Helper functions for encoding and decoding messages
    static std::string jsonify_data(std::string mac_address, serial_pose2D_t odom);
    static std::string mac_bytes_to_string(const mac_address_t mac_address);
    static void mac_string_to_bytes(const std::string &mac_str, mac_address_t mac_address);
    static void read_bytes(uint8_t *buffer, uint16_t len);
    static void read_mac_address(uint8_t *mac_address, uint16_t *pkt_len);
    static void read_message(uint8_t *data_serialized, uint16_t message_len, uint8_t *data_checksum);
    static int validate_message(uint8_t *data_serialized, uint16_t message_len, uint8_t data_checksum);
    template <typename T>
    static void encode_and_push_msg(T *msg, uint16_t topic, mac_address_t mac_address);
    static uint8_t checksum(uint8_t *addends, int len);
};

std::vector<std::string> get_macs_from_file(const std::string &file_name);

template<typename T=mbot, typename... Args>
std::vector<T> init_from_file(const std::string & file_name, Args&&... args)
{
    // Given a file with a list of MAC addresses, return an array of mbots
    std::ifstream file(file_name);
    if (!file.is_open())
    {
        perror("Error opening MAC address file. Ensure specified path is correct.\n");
        return std::vector<T>();
    }
    std::string line;

    // Read the file and get the number of bots and the mac addresses
    int num_bots = 0;
    std::vector<std::string> macs;
    while (std::getline(file, line))
    {
        if (line[0] == '#')
            continue;
        macs.push_back(line.substr(0, 17));
        num_bots++;
    }

    // Create the mbots
    std::vector<T> mbots;
    mbots.reserve(num_bots);
    for (int i = 0; i < num_bots; i++)
    {
        std::string name = "mbot-" + std::to_string(i);
        mbots.emplace_back(name, macs[i], std::forward<Args>(args)...);
    }

    return mbots;
}

#endif