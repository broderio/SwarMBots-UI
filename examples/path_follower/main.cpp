#include "mbot.h"
#include <iostream>
#include <unistd.h>
#include <fstream>

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <file_path>\n";
        return 1;
    }
    std::string port = argv[1];
    std::string file_path = argv[2];

    mbot::port = port;
    std::ifstream file(file_path);
    std::string mac_str;
    std::getline(file, mac_str);
    std::cout << "Mac address: " << mac_str << "\n";

    // Create mbot object
    mbot m("mbot", mac_str);

    // P-Controller for angular velocity
    float Kp = 0.75;
    float wz_goal = 2;
    float wz = wz_goal;
    while (1) {
        m.set_robot_vel_goal(0, 0, wz);
        sleep(2);
        serial_twist2D_t vel = m.get_robot_vel();
        float error = wz_goal - vel.wz;
        wz += Kp * error;
        std::cout << "Current angular velocity: " << vel.wz << "\n";
        std::cout << "Error: " << error << "\n";
    }
}