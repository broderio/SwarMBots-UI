#include "mbot.h"
#include <iostream>
#include <unistd.h>
using std::cout;
using std::cin;

int main() {
    mbot_params_t mbot_params;
    mbot::port = "/dev/cu.usbserial-14210"; // This works on macOS

    std::vector<mbot> mbot_list = mbot::init_from_file("/Users/broderio/Repositories/SwarMBots-UI/macs.txt");
    while(1){
        cout << "Turning right\n";
        for (mbot &m : mbot_list) {
            m.set_robot_vel_goal(0,0,5);
        }
        sleep(5);

        cout << "Stopping\n";
        for (mbot &m : mbot_list) {
            m.set_robot_vel_goal(0,0,0);
        }
        sleep(5);

        cout << "Turning left\n";
        for (mbot &m : mbot_list) {
            m.set_robot_vel_goal(0,0,-5);
        }
        sleep(5);

        cout << "Stopping\n";
        for (mbot &m : mbot_list) {
            m.set_robot_vel_goal(0,0,0);
        }
        sleep(5);
    }
}