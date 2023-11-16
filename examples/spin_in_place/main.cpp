#include "mbot.h"
#include <iostream>
#include <unistd.h>
using std::cout;
using std::cin;

int main() {
    mbot_params_t mbot_params;
    mbot::port = "/dev/cu.usbserial-14110"; // This works on macOS

    std::vector<mbot> mbot_list = mbot::init_from_file();

    serial_twist2D_t curr_vel;
    char ans = 'n';
    // while (ans != 'y'){
    //     cout << "ready? [y/n]\n";
    //     cin >> ans;
    // }

    while(1){
        cout << "Gerald turning right\n";
        mbot_list[0].set_robot_vel_goal(0,0,-5);
        sleep(5);

        cout << "stopping\n";
        mbot_list[0].set_robot_vel_goal(0,0,0);
        sleep(5);

        cout << "Gerald left\n";
        mbot_list[0].set_robot_vel_goal(0,0,5);
        sleep(5);

        cout << "Gerald stopping\n";
        mbot_list[0].set_robot_vel_goal(0,0,0);
        sleep(5);
    }
}