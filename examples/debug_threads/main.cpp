#include "mbot.h"
#include <iostream>
#include <unistd.h>
using std::cout;
using std::cin;

// TODO: Make serial port and file path for macs.txt an input argument
int main() {
    mbot::port = "/dev/cu.usbserial-14130"; // This works on macOS

    std::vector<mbot> mbot_list = mbot::init_from_file("/Users/broderio/Repositories/SwarMBots-UI/macs.txt");
    mbot::set_verbose();
    std::cout << "Number of mbot objects: " << mbot_list.size() << "\n";
    while (1) {
        sleep(1);
    }
}