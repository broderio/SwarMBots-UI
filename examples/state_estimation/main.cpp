#include <iostream>
#include <unistd.h>

#include "comp_filter.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>\n";
        return 1;
    }
    std::string port = argv[1];

    mbot::init(port);
    mbot::set_verbose(true);
    
    std::vector<std::string> macs = get_macs_from_file("macs.txt");
    comp_filter c0("c0", macs[0], 0.98);
    comp_filter c1("c1", macs[1], 0.98);
    comp_filter c2("c2", macs[2], 0.98);
    usleep(1000000); // Sleep for 1s

    c0.set_filtered_pose(-0.5, 0.0, M_PI / 2.0);
    c0.reset_encoders();

    c1.set_filtered_pose(0.0, 0.0, M_PI / 2.0);
    c1.reset_encoders();

    c2.set_filtered_pose(0.5, 0.0, M_PI / 2.0);
    c2.reset_encoders();

    comp_filter::start_server();

    while(1){
        usleep(100000); // Sleep for 100ms
    }
}