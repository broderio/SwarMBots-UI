#include <iostream>
#include <unistd.h>
#include <fstream>
#include <csignal>
#include <queue>
#include <cmath>

#include "mbot.h"
#include "chaser.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> \n";
        return 1;
    }
    std::string port = argv[1];
    std::string path_file_path = argv[3];

    // Set serial port
    mbot::init(port);

    // Set fast mode
    // chaser::set_fast(true);

    // Get mac addresses
    std::vector<std::string> macs = get_macs_from_file("macs.txt");

    // Create mbot object
    mbot m("mbot", macs[0]);

    // Create chaser objects
    chaser c1("chaser-1", macs[1], 0.5, 0.05);
    // chaser c2("chaser-2", macs[2], 0.25, 0.05);

    usleep(1000000); // Wait for odometry to reset
    
    float x = m.get_odom().x;
    while (x != 0.0) {
        m.reset_odom();
        usleep(100000);
        std::cout << x << '\n';
        x = m.get_odom().x;
    }

    while (c1.get_odom().x != -0.5) {
        c1.set_odom(-0.5, 0.0, 0.0);
        usleep(100000);
    }
    // c2.set_odom(-0.5, -0.25, 0.0);
    usleep(1000000); // Wait for odometry to reset

    chaser::start_server();

    std::signal(SIGINT, [](int signum) {
        flag = 0;
    });

    // Record odometry until user presses enter
    std::cout << "Chaser!\n";
    std::cout << "Put host into serial mode ...\n";
    std::cout << "Press enter to start driving\n";
    std::cin.get();
    std::cout << "Driving along path ...\n";
    std::cout << "Press ctrl-c to stop\n";

    // Follow path
    serial_pose2D_t goal = m.get_odom();
    c1.set_goal_pose(goal);
    c1.set_ready(true);
    // c2.set_goal_pose(goal);
    while (flag) {
        m.set_robot_vel_goal(0.12, 0.0, -0.28);
        goal = m.get_odom();
        c1.set_goal_pose(goal);
        // c2.set_goal_pose(goal);
        usleep(100000);
    }
    m.set_robot_vel_goal(0.0, 0.0, 0.0);
    std::cout << "Done following!\n";
    exit(0);
}