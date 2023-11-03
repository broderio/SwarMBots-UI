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
class Robot{
public:
    // Initializes the robot
    int robot_init(mbot_params_t params, uint8_t *mac_address);

    // Gets the odometry
    int robot_get_odometry(serial_pose2d_t *odometry);

    // Gets the IMU
    int robot_get_imu(serial_mbot_imu_t *imu);

    // Gets the encoders
    int robot_get_encoders(serial_mbot_encoders_t *encoders);

    // Gets the velocity
    int robot_get_vel(serial_twist2d_t *velocity);

    // Gets the motor velocity
    int robot_get_motor_vel(serial_mbot_motor_vel_t *motor_vel);

    // Gets the motor PWM
    int robot_get_motor_pwm(serial_mbot_motor_pwm_t *motor_pwm);

    // Gets the drive mode
    int robot_get_drive_mode(int *drive_mode);

    // Resets the odometry
    int robot_reset_odometry();

    // Resets the encoders
    int robot_reset_encoders();

    // Sets the velocity
    int robot_set_vel(float linear, float angular);

    // Sets the drive mode
    int robot_set_drive_mode(nt mode);

    // Sets the motor velocity if the drive mode is MODE_MOTOR_VEL_OL
    int robot_set_motor_vel(robot_t *r, float left, float right);

    // Sets the motor PWM if the drive mode is MODE_MOTOR_PWM
    int robot_set_motor_pwm(robot_t *r, float left, float right);

    // Callback function for reading data from the host via USB serial
    void data_read_host_cb(void* args);
    private:
    void mutex_wrapper(std::mutex);
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
};

#endif