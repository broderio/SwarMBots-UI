#include <stdlib.h>
#include <stdio.h>
#include <string>

#include "Robot.h"
#include "threads.h"
#include "mbot_lcm_msgs_serial.h"
#include "mbot_params.h"
#include "comms.h"

using std::mutex;
using std::thread;
using std::string;

int Robot::robot_init(mbot_params_t params, uint8_t *mac_address);

// Gets the odometry
int Robot::robot_get_odometry(serial_pose2d_t *odometry);

// Gets the IMU
int Robot::robot_get_imu(serial_mbot_imu_t *imu);

// Gets the encoders
int Robot::robot_get_encoders(serial_mbot_encoders_t *encoders);

// Gets the velocity
int Robot::robot_get_vel(serial_twist2d_t *velocity);

// Gets the motor velocity
int Robot::robot_get_motor_vel(serial_mbot_motor_vel_t *motor_vel);

// Gets the motor PWM
int Robot::robot_get_motor_pwm(serial_mbot_motor_pwm_t *motor_pwm);

// Gets the drive mode
int Robot::robot_get_drive_mode(int *drive_mode);

// Resets the odometry
int Robot::robot_reset_odometry();

// Resets the encoders
int Robot::robot_reset_encoders();

// Sets the velocity
int Robot::robot_set_vel(float linear, float angular);

// Sets the drive mode
int Robot::robot_set_drive_mode(int mode);

// Sets the motor velocity if the drive mode is MODE_MOTOR_VEL_OL
int Robot::robot_set_motor_vel(float left, float right);

// Sets the motor PWM if the drive mode is MODE_MOTOR_PWM
int Robot::robot_set_motor_pwm(float left, float right);