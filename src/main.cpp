#include "mbot/mbot.h"
#include "mbot_params.h"
#include "comms/comms.h"
#include <iostream>
using std::cout;
using std::cin;

int main() {
    mbot_params_t mbot_params;
    std::string g_mac_str = "EC:DA:3B:46:83:05";
    // uint8_t g[MAC_ADDR_LEN] = {0xEC, 0xDA, 0x3B, 0x46, 0x83, 0x05};
    //uint8_t h[MAC_ADDR_LEN] = {0x48,0x27,0xE2,0xFD,0x65,0xD1};
    // mbot::port_name = "COM17";
    mbot::port = "/dev/cu.usbserial-14110"; // This works on macOS
    mbot mbotG("Gerald", g_mac_str, mbot_params);
    //mbot mbotH("Henry", h, mbot_params);
    serial_twist2D_t curr_vel;
    char ans = 'n';
    while (ans != 'y'){
        cout << "ready? [y/n]\n";
        cin >> ans;
    }
    while(1){
        cout << "Gerald turning right\n";
        mbotG.set_robot_vel_goal(0,0,-5);
        sleep(5);

        cout << "stopping\n";
        mbotG.set_robot_vel_goal(0,0,0);
        sleep(5);

        cout << "Gerald left\n";
        mbotG.set_robot_vel_goal(0,0,5);
        sleep(5);

        cout << "Gerald stopping\n";
        mbotG.set_robot_vel_goal(0,0,0);
        sleep(5);
    }
}