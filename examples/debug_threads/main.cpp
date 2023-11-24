#include "mbot.h"
#include <iostream>
#include <unistd.h>

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <file_path>\n";
        return 1;
    }
    std::string port = argv[1];
    std::string file_path = argv[2];

    mbot::port = port;

    std::vector<mbot> mbot_list = mbot::init_from_file(file_path);
    mbot::set_verbose();
    mbot::start_server();
    std::cout << "Number of mbot objects: " << mbot_list.size() << "\n";
    usleep(5000000);
    exit(0);
}