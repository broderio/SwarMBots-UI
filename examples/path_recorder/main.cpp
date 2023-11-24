#include "mbot.h"
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <csignal>

volatile sig_atomic_t flag = 1;

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
    mbot m("mbot", mac_str);
    mbot::start_server();
    m.reset_odom();
    m.reset_encoders();
    usleep(1000000); // Wait for odometry to reset (1s

    // Open path file
    std::ofstream path_file(path_file_path);
    std::signal(SIGINT, [](int signum) {
        flag = 0;
    });

    // Record odometry until user presses enter
    std::cout << "Path Recorder!\n";
    std::cout << "Put host into pilot mode ...\n";
    std::cout << "Press enter to start recording odometry\n";
    std::cin.get();
    std::cout << "Recording odometry ...\n";
    std::cout << "Press ctrl-c to stop recording\n";
    while (flag) {
        serial_pose2D_t odom = m.get_odom();
        path_file << odom.utime << " " << odom.x << " " << odom.y << " " << odom.theta << "\n";
        usleep(250000); // 4Hz
    }
    path_file.close();
    std::cout << "Done recording!\n";
    exit(0);
}