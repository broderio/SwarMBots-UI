#ifndef ROBOT_H
#define ROBOT_H

#include "mbot_lcm_msgs_serial.h"
#include "mbot_params.h"
#include "comms.h"
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

class robot_t{
public:
    robot_t();
    ~robot_t();
    // Initializes the robot
    int robot_init(mbot_params_t params, uint8_t *mac_address);

    // Gets the odometry
    int robot_get_odometry(serial_pose2d_t &odometry);

    // Gets the IMU
    int robot_get_imu(serial_mbot_imu_t &imu);

    // Gets the encoders
    int robot_get_encoders(serial_mbot_encoders_t &encoders);

    // Gets the velocity
    int robot_get_vel(serial_twist2d_t &velocity_out);

    // Gets the motor velocity
    int robot_get_motor_vel(serial_mbot_motor_vel_t &motor_vel);

    // Gets the motor PWM
    int robot_get_motor_pwm(serial_mbot_motor_pwm_t &motor_pwm);

    // Gets the drive mode
    int robot_get_drive_mode(int &drive_mode);

    // Resets the odometry
    int robot_reset_odometry();

    // Resets the encoders
    int robot_reset_encoders();

    // Sets the velocity
    int robot_set_vel(float linear, float angular);

    // Sets the drive mode
    int robot_set_drive_mode(int mode);

    // Sets the motor velocity if the drive mode is MODE_MOTOR_VEL_OL
    int robot_set_motor_vel(float left, float right);

    // Sets the motor PWM if the drive mode is MODE_MOTOR_PWM
    int robot_set_motor_pwm(float left, float right);

    // Callback function for reading data from the host via USB serial
    void data_read_host_cb(void* args);

    private:
    //  Mutexes to protect each member variable to avoid race conditions
    //  when the user tries to read a variable as its being written.
    std::mutex odometry_mutex;
    std::mutex velocity_mutex;
    std::mutex imu_mutex;
    std::mutex encoders_mutex;
    std::mutex motor_vel_goal_mutex;
    std::mutex motor_vel_mutex;
    std::mutex motor_pwm_mutex;
    std::mutex drive_mode_mutex;

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

/*
I feel like we need to wrap another layer over this to launch a task to intake USB packets and then route them.
It also might contain an init funtion that the user has to call at the beginning of their main function
to initialize the USB packet routing task. I'm also thinking what we might actually want to expose to the user
is an array of robots 
*/

#endif