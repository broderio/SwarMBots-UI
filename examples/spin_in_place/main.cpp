#include "mbot.h"
#include <iostream>
#include <unistd.h>

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>\n";
        return 1;
    }
    std::string port = argv[1];

    mbot::port = port;

    std::vector<mbot> mbot_list = mbot::init_from_file();
    while(1){
        std::cout << "Turning right\n";
        for (mbot &m : mbot_list) {
            m.set_robot_vel_goal(0,0,5);
        }
        sleep(5);

        std::cout << "Stopping\n";
        for (mbot &m : mbot_list) {
            m.set_robot_vel_goal(0,0,0);
        }
        sleep(5);

        std::cout << "Turning left\n";
        for (mbot &m : mbot_list) {
            m.set_robot_vel_goal(0,0,-5);
        }
        sleep(5);

        std::cout << "Stopping\n";
        for (mbot &m : mbot_list) {
            m.set_robot_vel_goal(0,0,0);
        }
        sleep(5);
    }
}