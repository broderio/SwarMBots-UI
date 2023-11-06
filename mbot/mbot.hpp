#include <thread>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <string>

#include <iomanip>
#include <sstream>

#include "mbot_lcm_msgs_serial.h"
#include "mbot_params.h"
#include "comms.h"

#define MAC_LENGTH 12

using mac_address_t = uint8_t[MAC_LENGTH];

class mbot
{
public:
    mbot(const std::string &name, const mac_address_t mac_address, const mbot_params_t &params);
    ~mbot();

    serial_twist2D_t get_robot_vel();
    serial_mbot_imu_t get_imu();
    serial_mbot_motor_vel_t get_motor_vel();

    serial_twist2D_t get_robot_vel_goal();
    serial_mbot_motor_vel_t get_motor_vel_goal();
    int set_robot_vel_goal(float vx, float vy, float wz);
    int set_motor_vel_goal(float a, float b, float c = 0.0f);

    serial_mbot_motor_pwm_t get_motor_pwm();
    int set_motor_pwm(float a, float b, float c=0.0f);

    serial_pose2D_t get_odom();
    serial_mbot_encoders_t get_encoders();
    int set_odom(float x, float y, float theta);
    int reset_odom();
    int set_encoders(int a, int b, int c = 0);
    int reset_encoders();

    drive_mode_t drive_mode;
    mbot_params_t params;
    mac_address_t mac_address;
    const std::string mac_str;
    std::string name;
    int is_alive;

private:
    serial_twist2D_t robot_vel;
    serial_mbot_imu_t imu;
    serial_mbot_motor_vel_t motor_vel;
    serial_mbot_motor_pwm_t motor_pwm;

    serial_twist2D_t robot_vel_goal;
    serial_mbot_motor_vel_t motor_vel_goal;
    serial_mbot_motor_pwm_t motor_pwm_goal;

    // Mutexes for data
    std::mutex robot_vel_mutex;
    std::mutex imu_mutex;
    std::mutex motor_vel_mutex;
    std::mutex motor_pwm_mutex;

    // Mutex for USB
    std::mutex usb_mutex;

    // Static variables and functions for robot_thread()
    std::string mac_to_string(const mac_address_t mac_address); // converts mac_address_t to std::string
    static std::unordered_map<std::string, mbot *> mbots; // contains pointers to all instatiated mbot objects
    static void mbot_th(); // updates all instatiated mbot objects
    static bool th_running; // set true on the first instatiated mbot object
    std::thread mbot_th_handle;
};
