#include "mbot.h"
#include <iostream>
#include <unistd.h>
using std::cout;
using std::cin;

// TODO: Make serial port and file path for macs.txt an input argument
int main() {
    mbot_params_t mbot_params;
    mbot::port = "/dev/cu.usbserial-14210"; // This works on macOS

    std::vector<mbot> mbot_list = mbot::init_from_file("/Users/broderio/Repositories/SwarMBots-UI/macs.txt");
    std::cout << "Number of mbot objects: " << mbot_list.size() << "\n";
    while (1) {
        sleep(1);
    }
}