#include <iostream>
#include <unistd.h>
#include <fstream>

#include "mbot.h"

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <file_path>\n";
        return 1;
    }
    std::string port = argv[1];
    std::string file_path = argv[2];

    mbot::port = port;
    std::ifstream file(file_path);
    std::string mac_str;
    std::getline(file, mac_str);
    std::cout << "MAC address: " << mac_str << "\n";

    // Create mbot object
    mbot m("mbot", mac_str);
    mbot::start_server();
    
    m.reset_odom();

    while (1) {
        sleep(1);
    }
}