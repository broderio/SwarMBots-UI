#include "mbot.hpp"

using mac_address_t = uint8_t[MAC_LENGTH];

/* thread_safe_t member function definitions */

template <typename T>
thread_safe_t<T>::thread_safe_t()
{
    data = T();
}

template <typename T>
thread_safe_t<T>::~thread_safe_t() {}

template <typename T>
T thread_safe_t<T>::get()
{
    std::lock_guard<std::mutex> lock(mtx);
    return data;
}

template <typename T>
void thread_safe_t<T>::set(T data)
{
    std::lock_guard<std::mutex> lock(mtx);
    this->data = data;
}

/* mbot member function definitions */

// Set static bool false for initial constructor
bool mbot::th_running = false;

// Constructor for mbot class. This function is not thread safe! Mbots should not be instantiated concurrently.
mbot::mbot(const std::string &name, const mac_address_t mac_address, const mbot_params_t &params)
{
    this->name = name;
    std::memcpy(this->mac_address, mac_address, MAC_LENGTH);
    this->params = params;
    this->is_alive = true;

    // Add the robot to the swarm
    mbots.insert(std::pair<std::string, mbot *>(mac_str, this));

    // Start the thread
    if (!th_running)
    {
        th_running = true;
        mbot_th_handle = std::thread(&mbot::mbot_th, this);
    }
}

// Destructor for mbot class. This function is not thread safe! Mbots should be not be freed concurrently.
mbot::~mbot()
{
    // Remove the robot from the swarm
    std::string mac_str = mac_to_string(mac_address);
    mbots.erase(mac_str);

    // Stop the thread if there are no more robots
    if (mbots.size() == 0)
    {
        th_running = false;
        mbot_th_handle.join();
    }
}

// Converts uin8_t[MAC_LENGTH] to a string where each 2 bytes are separated by semicolons
std::string mbot::mac_to_string(const mac_address_t mac_address)
{
    std::stringstream ss;
    for (int i = 0; i < 12; ++i)
    {
        if (i > 0 && i % 2 == 0)
        {
            ss << ':';
        }
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(mac_address[i]);
    }
    return ss.str();
}

serial_twist2D_t mbot::get_robot_vel() {
    return this->robot_vel.get();
}

serial_mbot_imu_t mbot::get_imu() {

}

serial_mbot_motor_vel_t mbot::get_motor_vel() {

}

serial_twist2D_t mbot::get_robot_vel_goal() {

}

serial_mbot_motor_vel_t mbot::get_motor_vel_goal() {

}

int mbot::set_robot_vel_goal(float vx, float vy, float wz) {

}

int mbot::set_motor_vel_goal(float a, float b, float c = 0.0f) {

}

serial_mbot_motor_pwm_t mbot::get_motor_pwm() {

}

int mbot::set_motor_pwm(float a, float b, float c = 0.0f) {

}

serial_pose2D_t mbot::get_odom() {

}

serial_mbot_encoders_t mbot::get_encoders() {

}

int mbot::set_odom(float x, float y, float theta) {

}

int mbot::reset_odom() {

}

int mbot::set_encoders(int a, int b, int c = 0) {

}

int mbot::reset_encoders() {

}

void mbot::mbot_th() {

}