#include "mbot.h"

int main() {
    mbot::port = "/dev/cu.usbserial-14110"; // This works on macOS

    // Get first mac address from file
    std::ifstream file("/Users/broderio/Repositories/SwarMBots-UI/macs.txt");
    std::string mac_str;
    std::getline(file, mac_str);
    std::cout << "MAC address: " << mac_str << "\n";

    // Create mbot object
    mbot m("mbot", mac_str);
    
    m.reset_odom();
    m.start_server();

    while (1) {
        sleep(1);
    }
}