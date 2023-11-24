#include "mbot.h"
#include <iostream>
#include <unistd.h>

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>\n";
        return 1;
    }
    std::string port = argv[1];

    mbot::port = port;

    std::vector<mbot> mbot_list = mbot::init_from_file();
    mbot::set_verbose();
    mbot::start_server();
    std::cout << "Number of mbot objects: " << mbot_list.size() << "\n";
    while (true) 
        usleep(100000);
    exit(0);
}