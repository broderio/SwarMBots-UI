#include <thread>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <string>
#include <queue>

#include <iomanip>
#include <sstream>

#include "../mbot_lcm_msgs_serial/mbot_lcm_msgs_serial.h"
#include "../mbot_params.h"
#include "../comms/comms.h"

#define MAC_LENGTH 12

using mac_address_t = uint8_t[MAC_LENGTH];

class mbot
{
public:
    mbot();
    mbot(const std::string &name, const mac_address_t mac_address, const mbot_params_t &params);
    ~mbot();

    // NOTE: Changed many outputs of type int to type void (why are setters returning?)
    serial_twist2D_t get_robot_vel();
    serial_mbot_imu_t get_imu();
    serial_mbot_motor_vel_t get_motor_vel();

    serial_twist2D_t get_robot_vel_goal();
    serial_mbot_motor_vel_t get_motor_vel_goal();
    void set_robot_vel_goal(float vx, float vy, float wz);
    void set_motor_vel_goal(float a, float b, float c);
    // no motor_pwm_goal stuff?

    serial_mbot_motor_pwm_t get_motor_pwm();
    void set_motor_pwm(float a, float b, float c);

    serial_pose2D_t get_odom();
    serial_mbot_encoders_t get_encoders();
    void set_odom(float x, float y, float theta);
    void reset_odom(); // why did this return int? setting to void...
    void set_encoders(int a, int b, int c);
    void reset_encoders();

    drive_mode_t drive_mode;
    mbot_params_t params;
    mac_address_t mac_address;
    const std::string mac_str;
    std::string name;
    int is_alive;

private:
    // Thread safe class
    template <typename T>
    class thread_safe_t
    {
    public:
        thread_safe_t();
        ~thread_safe_t();
        T get();
        void set(T data);

    private:
        T data;
        std::mutex mtx;
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

    // Packet object
    class packet_t
    {
    public:
        packet_t();
        ~packet_t();
        uint8_t *data;
        uint8_t length;
        mac_address_t mac_address;
    };

    // Static variables and functions for robot_thread()
    std::string mac_to_string(const mac_address_t mac_address); // converts mac_address_t to std::string
    static std::unordered_map<std::string, mbot *> mbots;       // contains pointers to all instatiated mbot objects

    static thread_safe_t<int> num_mbots; // total  number of instatiated mbots

    static void mbot_th();  // updates all instatiated mbot objects **Shouldn't this read USB and alert Mbots?
    static bool th_running; // set true on the first instatiated mbot object
    static std::thread mbot_th_handle;
    static int serial_port;
    static std::string port_name;

    static void send_th(); // thread to send updates to host via serial
    static std::thread send_th_handle;
    static std::mutex send_mutex;
    static std::queue<packet_t> send_queue; // queue containing packets to be sent NOTE: pointer to packet and pointer to packet length sequentially
    static std::condition_variable send_cv; // condition variable to waken the send thread

    void data_th(); // thread for each robot to parse, process, and update variables
    std::thread data_th_handle;
};
