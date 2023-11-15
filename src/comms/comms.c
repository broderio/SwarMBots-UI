#include "comms.h"

uint8_t checksum(uint8_t* addends, int len) {
    //takes in an array and sums the contents then checksums the array
    int sum = 0;
    for (int i = 0; i < len; i++) {
        sum += addends[i];
    }
    return 255 - ( ( sum ) % 256 );
}

void read_mac_address(int serial_port, uint8_t* mac_address, uint16_t* pkt_len) {
    uint8_t trigger_val = 0x00;
    while(trigger_val != 0xff)
    {
        read(serial_port, &trigger_val, 1);
    }
    read(serial_port, pkt_len, 2);
    read(serial_port, mac_address, MAC_ADDR_LEN);
}

void read_header(int serial_port, uint8_t* header_data) {
    read(serial_port, header_data, ROS_HEADER_LEN);
}

void read_message(int serial_port, uint8_t* msg_data_serialized, uint16_t message_len, char* topic_msg_data_checksum) {
    read(serial_port, msg_data_serialized, message_len);
    read(serial_port, topic_msg_data_checksum, 1);
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
int validate_message(uint8_t* msg_data_serialized, uint16_t message_len, char topic_msg_data_checksum) {
    uint8_t cs_topic_msg_data = checksum(msg_data_serialized, message_len); 
    int valid_message = (cs_topic_msg_data == topic_msg_data_checksum);
    return valid_message;
}

int validate_mac_address(uint8_t* mac_address, uint8_t checksum_val) {
    return checksum(mac_address, MAC_ADDR_LEN) == checksum_val;
}

void encode_msg(uint8_t* msg, int msg_len, uint16_t topic, uint8_t mac_address[6], uint8_t* msg_ser, int msg_ser_len) {
    // check to make sure lengths align
    if (MAC_ADDR_LEN + 3 + msg_len + ROS_PKG_LEN != msg_ser_len) {
        printf("Error: The length of the serialized message array does not match the length of the message array plus packaging.\n");
        return;
    }

    // add mac address
    msg_ser[0] = SYNC_FLAG;
    msg_ser[1] = (uint8_t) ((msg_len + ROS_PKG_LEN) % 255);
    msg_ser[2] = (uint8_t) ((msg_len + ROS_PKG_LEN) >> 8);
    memcpy(msg_ser + 3, mac_address, MAC_ADDR_LEN);

    // add ROS packet header
    msg_ser[9] = SYNC_FLAG;
    msg_ser[10] = VERSION_FLAG;
    msg_ser[11] = (uint8_t) (msg_len % 255);
    msg_ser[12] = (uint8_t) (msg_len >> 8);
    uint8_t cs1_addends[2] = {msg_ser[11], msg_ser[12]};
    msg_ser[13] = checksum(cs1_addends, 2);

    // add topic and message
    msg_ser[14] = (uint8_t) (topic % 255);
    msg_ser[15] = (uint8_t) (topic >> 8);
    memcpy(msg_ser + 16, msg, msg_len);
    uint8_t cs2_addends[msg_len + 2];
    cs2_addends[0] = msg_ser[14];
    cs2_addends[1] = msg_ser[15];
    memcpy(cs2_addends + 2, msg, msg_len);
    msg_ser[16 + msg_len] = checksum(cs2_addends, msg_len + 2);
}