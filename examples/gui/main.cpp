#include <iostream>
#include <vector>
#include <unistd.h>
#include <fstream>

#include "mbot.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>\n";
        return 1;
    }
    std::string port = argv[1];

    // Create mbot object
    mbot::port = port;
    std::vector<mbot> mbots = mbot::init_from_file();
    
    // mbots[0].set_odom(0, 0.5, 0);
    mbots[0].reset_odom();
    mbots[0].reset_encoders();
    // mbots[2].set_odom(0, -0.5, 0);

    mbot::start_server();

    while (1) {
        sleep(1);
    }
}