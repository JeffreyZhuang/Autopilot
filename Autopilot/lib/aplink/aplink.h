#ifndef LIB_APLINK_APLINK_H_
#define LIB_APLINK_APLINK_H_

#include "aplink_types.h"
#include <stdio.h>
#include <string.h>

// Checksum configuration
static constexpr uint16_t CRC16_POLY = 0x8005;  // CRC-16-IBM polynomial
static constexpr uint16_t CRC16_INIT = 0xFFFF;  // Initial value

bool aplink_parse_byte(aplink_msg* link_msg, uint8_t byte);
uint16_t aplink_pack(uint8_t packet[], const uint8_t payload[], const uint8_t payload_len, const uint8_t msg_id);
bool aplink_unpack(const uint8_t packet[], uint8_t payload[], uint8_t& payload_len, uint8_t& msg_id);
uint16_t aplink_calc_packet_size(uint8_t payload_size);
uint16_t aplink_crc16(const uint8_t data[], size_t length);

// Template for packing any message type
template<typename T>
inline uint16_t aplink_msg_pack(const T& data, uint8_t packet[], uint8_t msg_id) {
    uint8_t buffer[sizeof(data)];
    memcpy(buffer, &data, sizeof(data));
    return aplink_pack(packet, buffer, sizeof(buffer), msg_id);
}

// Template for decoding any message type
template<typename T>
inline bool aplink_msg_decode(aplink_msg* msg, T* output, uint8_t expected_msg_id) {
    if (msg->msg_id == expected_msg_id && msg->payload_len == sizeof(T)) {
        memcpy(output, msg->payload, sizeof(T));
        return true;
    }
    return false;
}

// Macro to declare message-specific functions
#define DECLARE_APLINK_MESSAGE(MSG_TYPE, MSG_ID) \
    inline uint16_t MSG_TYPE##_pack(const MSG_TYPE& data, uint8_t packet[]) { \
        return aplink_msg_pack(data, packet, MSG_ID); \
    } \
    inline bool MSG_TYPE##_decode(aplink_msg* msg, MSG_TYPE* output) { \
        return aplink_msg_decode(msg, output, MSG_ID); \
    }

#endif /* LIB_APLINK_APLINK_H_ */
