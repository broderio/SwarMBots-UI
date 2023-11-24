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
    float speeds[4] = {5.0, 0.0, -5.0, 0.0};
    int i = 0;
    while (1)
    {
        std::cout << "Setting wz to " << speeds[i] << "\n";
        for (mbot &m : mbot_list)
        {
            m.set_robot_vel_goal(0, 0, speeds[i]);
        }

        sleep(3);

        for (mbot &m : mbot_list)
        {
            serial_twist2D_t vel = m.get_robot_vel();
            std::cout << m.name << ": vx: " << vel.vx << ", vy: " << vel.vy << ", wz: " << vel.wz << "\n";
        }
        std::cout << "\n";
        i = (i + 1) % 4;
    }
}