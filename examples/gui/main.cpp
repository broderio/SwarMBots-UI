#include <iostream>
#include <vector>
#include <unistd.h>
#include <fstream>
#include <cmath>

#include "mbot.h"

/*
*   This example is a simple demo of the GUI. The robots are meant to be piloted
*   using the controller. The GUI can be accessed by opening gui.html in your 
*   browser. The robots should be placed 0.5m apart with robot 1 in the middle.
*   The robots' heading should be +90 degrees (facing the +y direction)
*/

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>\n";
        return 1;
    }
    std::string port = argv[1];

    // Create mbot object
    mbot::init(port);
    mbot::set_verbose(true);
    
    std::vector<mbot> mbots = init_from_file<mbot>("macs.txt");
    usleep(1000000); // Sleep for 1s

    float x = -0.5;
    for (mbot &m : mbots) {
        m.set_odom(x, 0.0, M_PI / 2.0);
        m.reset_encoders();
        x += 0.5;
    }

    mbot::start_server();

    while (1) {
        sleep(1);
    }
}