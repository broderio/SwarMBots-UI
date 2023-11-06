#include "mbot.hpp"

using mac_address_t = uint8_t[MAC_LENGTH];

/* mbot member function definitions */

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