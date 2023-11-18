#ifndef MBOT_H
#define MBOT_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <string>
#include <queue>

#include <iomanip>
#include <sstream>

#include "msgtypes.h"
#include "mbot_params.h"
#include "comms.h"

using mac_address_t = uint8_t[MAC_ADDR_LEN];

class mbot
{
public:
    mbot();
    mbot(const std::string &, const std::string &);
    ~mbot();
    mbot(const mbot &);

    static std::string port;

    static std::vector<mbot> init_from_file(const std::string &filename);

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

    drive_mode_t drive_mode;
    mbot_params_t params;
    mac_address_t mac_address;
    std::string name;
    int is_alive;

private:
    // Thread safe class
    template <typename T>
    class thread_safe_t
    {
    public:
        thread_safe_t();
        thread_safe_t(T data);
        ~thread_safe_t();
        T get();
        void set(T data);

    private:
        T data;
        std::mutex mtx;
    };

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

    thread_safe_t<serial_twist2D_t> robot_vel;
    thread_safe_t<serial_mbot_imu_t> imu;
    thread_safe_t<serial_mbot_motor_vel_t> motor_vel;
    thread_safe_t<serial_mbot_motor_pwm_t> motor_pwm;
    thread_safe_t<serial_pose2D_t> odom;
    thread_safe_t<serial_mbot_encoders_t> encoders;
    serial_twist2D_t robot_vel_goal;
    serial_mbot_motor_vel_t motor_vel_goal;

    // Mutex for USB
    std::mutex usb_mtx;

    void update_mbot(uint8_t *data);

    static std::string mac_to_string(const mac_address_t mac_address);                // converts mac_address_t to std::string
    static void string_to_mac(const std::string &mac_str, mac_address_t mac_address); // converts mac_address_t to std::string
    static std::unordered_map<std::string, mbot *> mbots;                             // contains pointers to all instatiated mbot objects

    static thread_safe_t<int> num_mbots; // total  number of instatiated mbots

    static thread_safe_t<bool> running; // set true on the first instatiated mbot object
    static void recv_th();              // updates all instatiated mbot objects **Shouldn't this read USB and alert Mbots?
    static std::thread mbot_th_handle;
    static int serial_port;

    static void send_th(); // thread to send updates to host via serial
    static std::thread send_th_handle;
    static std::mutex send_mutex;
    static std::queue<packet_t> send_queue; // queue containing packets to be sent
    static std::condition_variable send_cv; // condition variable to waken the send thread

    // Functions moved from comms.h
    static void read_mac_address(uint8_t* mac_address, uint16_t* pkt_len);
    static void read_message(uint8_t* data_serialized, uint16_t message_len, uint8_t* data_checksum);
    static int validate_message(uint8_t* data_serialized, uint16_t message_len, uint8_t data_checksum);
    static void encode_msg(uint8_t* msg, int msg_len, uint16_t topic, uint8_t mac_address[6], uint8_t* msg_ser, int msg_ser_len);
    static uint8_t checksum(uint8_t* addends, int len);

    // Other functions and members
    static thread_safe_t<uint64_t> start_time;
    static uint64_t get_time_millis();
};

#endif