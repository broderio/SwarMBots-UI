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

using mac_address_t = uint8_t[MAC_ADDR_LEN];

class mbot
{
public:
    mbot();
    mbot(const std::string &, const std::string &);
    mbot(const mbot &);
    ~mbot();

    static std::string port;

    static std::vector<mbot> init_from_file(const std::string &file_name="macs.txt");

    serial_twist2D_t get_robot_vel();
    serial_mbot_imu_t get_imu();
    serial_mbot_motor_vel_t get_motor_vel();
    serial_twist2D_t get_robot_vel_goal();
    void set_robot_vel_goal(float vx, float vy, float wz);
    serial_mbot_motor_vel_t get_motor_vel_goal();
    void set_motor_vel_goal(float a, float b, float c);
    serial_mbot_motor_pwm_t get_motor_pwm();
    void set_motor_pwm(float a, float b, float c);
    serial_pose2D_t get_odom();
    void set_odom(float x, float y, float theta);
    void reset_odom();
    serial_mbot_encoders_t get_encoders();
    void set_encoders(int a, int b, int c);
    void reset_encoders();
    void send_timesync();
    static void set_verbose();

    static bool is_running();

    static void start_server();

    static void on_update(std::function<void(mbot*)> callback);

    mac_address_t mac_address;
    std::string name;
    bool is_alive;

private:

    // Packet object
    class packet_t
    {
    public:
        packet_t();
        packet_t(const packet_t &);
        ~packet_t();
        packet_t &operator=(const packet_t &other);
        uint8_t *data;
        uint8_t length;
    };

    // Wrapper packet for serial data
    struct __attribute__((__packed__)) packets_wrapper_t {
        serial_mbot_encoders_t encoders;
        serial_pose2D_t odom;
        serial_mbot_imu_t imu;
        serial_twist2D_t mbot_vel;
        serial_mbot_motor_vel_t motor_vel;
        serial_mbot_motor_pwm_t motor_pwm;
    };
    static std::string jsonify_packets_wrapper(mac_address_t mac_address, packets_wrapper_t *packets_wrapper);

    std::atomic<serial_twist2D_t> robot_vel;
    std::atomic<serial_mbot_imu_t> imu;
    std::atomic<serial_mbot_motor_vel_t> motor_vel;
    std::atomic<serial_mbot_motor_pwm_t> motor_pwm;
    std::atomic<serial_pose2D_t> odom;
    std::atomic<serial_mbot_encoders_t> encoders;
    serial_twist2D_t robot_vel_goal;
    serial_mbot_motor_vel_t motor_vel_goal;

    void update_mbot(packets_wrapper_t *pkt);

    // User defined callback function for update
    std::function<void(mbot*)> update_cb;

    // Static variables and functions for robot_thread()
    static std::atomic<bool> verbose;
    
    static std::string mac_to_string(const mac_address_t mac_address); // converts mac_address_t to std::string
    static void string_to_mac(const std::string &mac_str, mac_address_t mac_address); // converts mac_address_t to std::string
    static std::unordered_map<std::string, mbot *> mbots;                             // contains pointers to all instatiated mbot objects

    static std::atomic<int> num_mbots; // total  number of instatiated mbots

    static std::atomic<bool> running; // set true on the first instatiated mbot object
    static void recv_th();              // updates all instatiated mbot objects
    static std::thread mbot_th_handle;
    static int serial_port;

    static void send_th(); // thread to send updates to host via serial
    static std::thread send_th_handle;
    static std::mutex send_mutex;
    static std::queue<packet_t> send_queue; // queue containing packets to be sent
    static std::condition_variable send_cv; // condition variable to waken the send thread

    // Functions moved from comms.h
    static void read_bytes(uint8_t* buffer, uint16_t len);
    static void read_mac_address(uint8_t* mac_address, uint16_t* pkt_len);
    static void read_message(uint8_t* data_serialized, uint16_t message_len, uint8_t* data_checksum);
    static int validate_message(uint8_t* data_serialized, uint16_t message_len, uint8_t data_checksum);
    static void encode_msg(uint8_t* msg, int msg_len, uint16_t topic, uint8_t mac_address[6], uint8_t* msg_ser, int msg_ser_len);
    static uint8_t checksum(uint8_t* addends, int len);

    // Server functions and members
    static telemetry_server server;
    static std::atomic<bool> server_running;
    static std::thread server_th_handle;

    // Other functions and members
    static std::atomic<uint64_t> start_time;
    static uint64_t get_time_millis();
};

#endif