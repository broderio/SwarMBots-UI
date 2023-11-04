#ifndef ROBOT_H
#define ROBOT_H

#include "mbot_lcm_msgs_serial.h"
#include "mbot_params.h"
#include "comms.h"
#include <thread>
#include <unordered_map>
#include <string>

/*
typedef struct robot_t {
    serial_pose2d_t odometry; // The robot's current odometry
    serial_twist2d_t velocity; // The robot's current velocity
    serial_mbot_imu_t imu; // The robot's current IMU values
    serial_mbot_encoders_t encoders; // The robot's current encoder values
    serial_mbot_motor_vel_t motor_vel_goal; // The robot's goal motor velocity
    serial_mbot_motor_vel_t motor_vel; // The robot's current motor velocity
    serial_mbot_motor_pwm_t motor_pwm; // The robot's current motor PWM
    int drive_mode; // The robot's current drive mode
    mbot_params_t params; // The robot's physical parameters
    uint8_t mac_address[MAC_LENGTH]; // The robot's MAC address for esp-now communication
} robot_t;
*/
#define MAC_LENGTH 12

struct USB_packet {
    uint8_t MAC_address[MAC_LENGTH];
    uint8_t *data;
    uint32_t data_len;
};

using std::mutex;
using std::unordered_map;
using std::condition_variable;
using std::string;

#define MAC_LENGTH 12
class Swarm{
    friend class robot_t;
    class robot_t{
        public:

        robot_t();
        robot_t(string);
        robot_t(string, int);
        ~robot_t();

        // Callback function for reading data from the host via USB serial
        void robot_thread(void* args);

        //  Mutexes to protect each member variable to avoid race conditions
        //  when the user tries to read a variable as its being written.
        mutex odometry_mutex;
        mutex velocity_mutex;
        mutex imu_mutex;
        mutex encoders_mutex;
        mutex motor_vel_goal_mutex;
        mutex motor_vel_mutex;
        mutex motor_pwm_mutex;
        mutex drive_mode_mutex;

        //condition variable to indicate when it's This robot's turn to digest the serial data packet
        condition_variable CV;

        serial_pose2d_t odometry; // The robot's current odometry
        serial_twist2d_t velocity; // The robot's current velocity
        serial_mbot_imu_t imu; // The robot's current IMU values
        serial_mbot_encoders_t encoders; // The robot's current encoder values
        serial_mbot_motor_vel_t motor_vel_goal; // The robot's goal motor velocity
        serial_mbot_motor_vel_t motor_vel; // The robot's current motor velocity
        serial_mbot_motor_pwm_t motor_pwm; // The robot's current motor PWM
        int drive_mode; // The robot's current drive mode
        mbot_params_t params; // The robot's physical parameters
        uint8_t mac_address[MAC_LENGTH]; // The robot's MAC address for esp-now communication
        int rob_error;
    };

    //this mutex protects the usb structure
    mutex usb_mutex;

    uint8_t num_robots;
    int swarm_error;
    USB_packet usb;
    unordered_map<string, robot_t> MAC_to_Robot;
    unordered_map<string, string> Name_to_MAC;
    //This process gets a new packet from USB, puts it in the USB
    void swarm_thread();
    robot_t* name_to_rob(string& rob_name);

    public:
    // Gets the odometry
    int robot_get_odometry(string &rob_name, serial_pose2d_t &odometry);

    // Gets the IMU
    int robot_get_imu(string &rob_name, serial_mbot_imu_t &imu);

    // Gets the encoders
    int robot_get_encoders(string &rob_name, serial_mbot_encoders_t &encoders);

    // Gets the velocity
    int robot_get_vel(string &rob_name, serial_twist2d_t &velocity_out);

    // Gets the motor velocity
    int robot_get_motor_vel(string &rob_name, serial_mbot_motor_vel_t &motor_vel);

    // Gets the motor PWM
    int robot_get_motor_pwm(string &rob_name, serial_mbot_motor_pwm_t &motor_pwm);

    // Gets the drive mode
    int robot_get_drive_mode(string &rob_name, int &drive_mode);

    // Resets the odometry
    int robot_reset_odometry(string &rob_name);

    // Resets the encoders
    int robot_reset_encoders(string &rob_name);

    // Sets the velocity
    int robot_set_vel(string &rob_name, float linear, float angular);

    // Sets the drive mode
    int robot_set_drive_mode(string &rob_name, int mode);

    // Sets the motor velocity if the drive mode is MODE_MOTOR_VEL_OL
    int robot_set_motor_vel(string &rob_name, float left, float right);

    // Sets the motor PWM if the drive mode is MODE_MOTOR_PWM
    int robot_set_motor_pwm(string &rob_name, float left, float right);

};


/*
I feel like we need to wrap another layer over this to launch a task to intake USB packets and then route them.
It also might contain an init funtion that the user has to call at the beginning of their main function
to initialize the USB packet routing task. I'm also thinking what we might actually want to expose to the user
is an array of robots 
*/

#endif