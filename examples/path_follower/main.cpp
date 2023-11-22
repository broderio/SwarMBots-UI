#include <iostream>
#include <unistd.h>
#include <fstream>
#include <csignal>
#include <queue>

#include "mbot.h"
#include "path_follower.h"

int main(int argc, char *argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <mac_file_path> <path_file_path>\n";
        return 1;
    }
    std::string port = argv[1];
    std::string mac_file_path = argv[2];
    std::string path_file_path = argv[3];

    // Set serial port
    mbot::port = port;

    // Get mac address
    std::ifstream macs(mac_file_path);
    std::string mac_str;
    std::getline(macs, mac_str);
    std::cout << "Mac address: " << mac_str << "\n";
    macs.close();

    // Create mbot object and start server
    path_follower m("mbot", mac_str, 1.0, 0.06);
    path_follower::start_server();
    std::signal(SIGINT, [](int signum) {
        flag = 0;
    });

    // Record odometry until user presses enter
    std::cout << "Path Follower!\n";
    std::cout << "Put host into serial mode ...\n";
    std::cout << "Press enter to start driving\n";
    std::cin.get();
    std::cout << "Driving along path ...\n";
    std::cout << "Press ctrl-c to stop\n";
    m.reset_odom();
    m.reset_encoders();
    usleep(1000000); // Wait for odometry to reset

    // Load path
    m.load_path(path_file_path);

    // Follow path
    m.follow_path();

    std::cout << "Done following!\n";
    exit(0);
}
