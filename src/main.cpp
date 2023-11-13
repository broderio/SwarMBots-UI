#include "mbot/mbot.h"
#include "mbot_params.h"
#include "comms/comms.h"
#include <iostream>
using std::cout;
using std::cin;

int main() {
    mbot_params_t mbot_params;
    uint8_t g[MAC_ADDR_LEN] = {0xF4,0x12,0xFA,0xFA,0x07,0x51};
    uint8_t h[MAC_ADDR_LEN] = {0x48,0x27,0xE2,0xFD,0x65,0xD1};
    mbot mbotG("Gerald", g, "COM12", mbot_params);
    mbot mbotH("Henry", h, "COM12", mbot_params);
    serial_twist2D_t curr_vel;
    char ans = 'n';
    while (ans != 'y'){
        cout << "ready? [y/n]\n";
        cin >> ans;
    }
    while(1){
        cout << "Henry turning left\n";
        sleep(100);
        mbotH.set_robot_vel_goal(0, 0, 5);
        cout << "Gerald turning right\n";
        sleep(100);
        mbotG.set_robot_vel_goal(0,0,-5);
        sleep(3000);
        cout << "stopping both\n";
        mbotH.set_robot_vel_goal(0,0,0);
        mbotG.set_robot_vel_goal(0,0,0);
        sleep(5000);
    }
}