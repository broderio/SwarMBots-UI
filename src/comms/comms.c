#include "comms.h"

uint8_t checksum(uint8_t* addends, int len) {
    //takes in an array and sums the contents then checksums the array
    int sum = 0;
    for (int i = 0; i < len; i++) {
        sum += addends[i];
    }
    return 255 - ( ( sum ) % 256 );
}


// Function to validate the header
int validate_header(uint8_t* header_data) {
    int valid_header = (header_data[1] == 0xfe);
    uint8_t cs1_addends[2] = {header_data[2], header_data[3]};
    uint8_t cs_msg_len = checksum(cs1_addends, 2);
    valid_header = valid_header && (cs_msg_len == header_data[4]);
    return valid_header;
}


// Function to validate the message
int validate_message(uint8_t* header_data, uint8_t* msg_data_serialized, uint16_t message_len, char topic_msg_data_checksum) {
    uint8_t cs2_addends[message_len + 2]; 
    cs2_addends[0] = header_data[5];
    cs2_addends[1] = header_data[6];
    for (int i = 0; i < message_len; i++) {
        cs2_addends[i + 2] = msg_data_serialized[i];
    }
    uint8_t cs_topic_msg_data = checksum(cs2_addends, message_len + 2); 
    int valid_message = (cs_topic_msg_data == topic_msg_data_checksum);
    return valid_message;
}

int encode_msg(uint8_t* MSG, int msg_len, uint16_t TOPIC, uint8_t* ROSPKT, int rospkt_len) {
    
    // SANITY CHECKS
    if (msg_len+ROS_PKG_LEN != rospkt_len) { 
        printf("%d != %d", msg_len+ROS_PKG_LEN, rospkt_len);
        printf("Error: The length of the ROSPKT array does not match the length of the MSG array plus packaging.\n");
        return 0;
    }


    // CREATE ROS PACKET
    //for ROS protocol and packet format see link: http://wiki.ros.org/rosserial/Overview/Protocol
    ROSPKT[0] = SYNC_FLAG;
    ROSPKT[1] = VERSION_FLAG;
    ROSPKT[2] = (uint8_t) (msg_len%255); //message length lower 8/16b via modulus and cast
    ROSPKT[3] = (uint8_t) (msg_len>>8); //message length higher 8/16b via bitshift and cast

    uint8_t cs1_addends[2] = {ROSPKT[2], ROSPKT[3]};
    ROSPKT[4] = checksum(cs1_addends, 2); //checksum over message length
    ROSPKT[5] = (uint8_t) (TOPIC%255); //message topic lower 8/16b via modulus and cast
    ROSPKT[6] = (uint8_t) (TOPIC>>8); //message length higher 8/16b via bitshift and cast

    for (int i = 0; i<msg_len; i++) { //write message bytes
        ROSPKT[i+7] = MSG[i];
    }
    
    uint8_t cs2_addends[msg_len+2]; //create array for the checksum over topic and message content
    cs2_addends[0] = ROSPKT[5];
    cs2_addends[1] = ROSPKT[6];
    for (int i = 0; i<msg_len; i++) {
        cs2_addends[i+2] = MSG[i];
    }

    ROSPKT[rospkt_len-1] = checksum(cs2_addends, msg_len+2); //checksum over message data and topic

    return 1;
}
