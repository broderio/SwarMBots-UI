#include "mbot.h"
#include <iostream>
#include <unistd.h>
#include <fstream>

using std::cout;
using std::cin;

// TODO: Make serial port and file path for macs.txt an input argument
int main() {
    mbot::port = "/dev/cu.usbserial-14210"; // This works on macOS

    // Get first mac address from file
    std::ifstream file("/Users/broderio/Repositories/SwarMBots-UI/macs.txt");
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