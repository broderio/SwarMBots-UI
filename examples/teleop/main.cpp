#include <iostream>

#include "mbot.h"
#include "teleop.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>\n";
        return 1;
    }
    std::string port = argv[1];

    // Create mbot object
    mbot::init(port);
    
    std::vector<teleop> mbots = init_from_file<teleop>("macs.txt");
    usleep(1000000); // Sleep for 1s

    std::cout << "Setting keys...\n";
    mbots[0].set_keys('w', 'a', 's', 'd', 'x');
    mbots[1].set_keys('t', 'f', 'g', 'h', 'b');
    mbots[2].set_keys('i', 'j', 'k', 'l', 'm');

    std::cout << "Starting teleop...\n";
    teleop::start_teleop();

    while (1) {
        sleep(1);
    }
}