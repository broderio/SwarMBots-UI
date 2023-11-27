#include <iostream>
#include <unistd.h>

#include "comp_filter.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>\n";
        return 1;
    }
    std::string port = argv[1];

    comp_filter::port = port;
    comp_filter cf("mbot-0", "ec:da:3b:46:83:05", 0.98);
    cf.reset_odom();
    cf.reset_encoders();
    usleep(1000000); // Sleep for 1s

    comp_filter::start_server();

    while(1){
        usleep(100000); // Sleep for 100ms
    }
}