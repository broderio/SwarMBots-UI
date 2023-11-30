#include <iostream>
#include <unistd.h>
#include <fstream>
#include <csignal>
#include <queue>

#include "mbot.h"
#include "path_follower.h"

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <path_file_path>\n";
        return 1;
    }
    std::string port = argv[1];
    std::string path_file_path = argv[2];

    // Set serial port
    mbot::init(port);

    // Get mac address
    std::ifstream macs("macs.txt");
    std::string mac_str;
    std::getline(macs, mac_str);
    std::cout << "Mac address: " << mac_str << "\n";
    macs.close();

    // Create mbot object and start server
    path_follower m("mbot", mac_str, 1.2, 0.1 );
    while (m.get_odom().x != 0.0) {
        m.reset_odom();
        usleep(100000);
    }
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

    // Load path
    m.load_path(path_file_path);

    // Follow path
    m.follow_path();

    std::cout << "Done following!\n";
    exit(0);
}
