#include <iostream>
#include <unistd.h>
#include <fstream>
#include <csignal>
#include <queue>
#include <cmath>

#include "mbot.h"
#include "chaser.h"

/*
*   This example demonstrates how to use the chaser class to make a robot follow another robot.
*   Place the robots into a triangle configuration with robot 0 at the top, robot 1 at the
*   bottom right, and robot 2 at the bottom left. Robot 0 will follow robot 1, robot 1 will
*   follow robot 2, and robot 2 will follow robot 0. Each robot should be oriented to look at
*   the robot it is following.
*/

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> \n";
        return 1;
    }
    std::string port = argv[1];

    // Set serial port
    mbot::init(port);

    // Set fast mode
    chaser::set_fast(true);

    // Create chaser objects
    std::vector<chaser> chasers = init_from_file<chaser>("macs.txt", 0.5, 0.05);

    // Initialize robots into an equilateral triangle centered at the origin
    chasers[0].set_odom(0, 0.5, -M_PI / 3.0);
    chasers[1].set_odom(0.5 * sqrt(3) / 2, -0.25, -M_PI);
    chasers[2].set_odom(-0.5 * sqrt(3) / 2, -0.25, M_PI / 3.0);

    chaser::start_server();

    std::signal(SIGINT, [](int signum) {
        flag = 0;
    });

    // Record odometry until user presses enter
    std::cout << "Chaser!\n";
    std::cout << "Put host into serial mode and press enter to begin \n";
    std::cin.get();
    std::cout << "Press ctrl-c to stop\n";

    // Init goal poses
    chasers[0].set_goal_pose(chasers[1].get_odom());
    chasers[1].set_goal_pose(chasers[2].get_odom());
    chasers[2].set_goal_pose(chasers[0].get_odom());

    // Start following
    for (chaser &c : chasers) {
        c.set_ready(true);
    }

    // Follow path
    while (flag) {
        for (int i = 0; i < 3; i++) {
            int j = (i + 1) % 3;
            serial_pose2D_t goal = chasers[j].get_odom();
            chasers[i].set_goal_pose(goal);
        }
        usleep(100000);
    }
    std::cout << "Done following!\n";
    exit(0);
}