#include "mbot.h"
#include <iostream>
#include <unistd.h>
using std::cout;
using std::cin;

// TODO: Make serial port and file path for macs.txt an input argument
int main() {
    mbot_params_t mbot_params;
    mbot::port = "/dev/cu.usbserial-14210"; // This works on macOS

    std::vector<mbot> mbot_list = mbot::init_from_file("/Users/broderio/Repositories/SwarMBots-UI/macs.txt");
    float speeds[4] = {5.0, 0.0, -5.0, 0.0};
    int i = 0;
    while(1){
        std::cout << "Setting wz to " << speeds[i] << "\n";
        for (mbot &m : mbot_list) {
            m.set_robot_vel_goal(0,0,speeds[i]);
        }

        sleep(3);

        for (mbot &m : mbot_list) {
            serial_twist2D_t vel = m.get_robot_vel();
            std::cout << m.name << ": vx: " << vel.vx << ", vy: " << vel.vy << ", wz: " << vel.wz << "\n";
        }
        std::cout << "\n";
        i = (i + 1) % 4;
    }
}