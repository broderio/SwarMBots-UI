#ifndef SWARM_H
#define SWARM_H

#include <robot_t.h>
#include <vector>
#include <unordered_map>

struct USB_packet {
    uint8_t MAC_address[MAC_LENGTH];
    int packet_type;
    //some way to store most recent USB data ?
};

class Swarm{
public:

    vector<robot_t> Robots; 
    /*
    What do we want to pass into inializer? Require that they pass in # of robots?
    Require that they pass in array of MAC addresses? Not sure what this will require,
    but this will at the very least launch the USB packet routing task
    */
    int Swarm_init();

    int add_robot(uint8_t *MAC_address);

private:
    //this mutex protects the usb structure
    std::mutex usb_mutex;

    int num_robots;
    int swarm_error;
    USB_packet usb;
    std::unordered_map<string, int> MAC_to_RobotID;
    //This process gets a new packet from USB, puts it in the USB
    int usb_scan_and_route();

};
#endif