#ifndef LIB_APLINK_APLINK_H_
#define LIB_APLINK_APLINK_H_

#include "aplink_types.h"
#include <stdio.h>
#include <string.h>

static constexpr uint8_t START_BYTE = 0xFE;

static constexpr uint8_t HEADER_LEN = 3;
static constexpr uint8_t FOOTER_LEN = 2;

static constexpr uint16_t MAX_PACKET_LEN = MAX_PAYLOAD_LEN + HEADER_LEN + FOOTER_LEN;

// Checksum configuration
static constexpr uint16_t CRC16_POLY = 0x8005;  // CRC-16-IBM polynomial
static constexpr uint16_t CRC16_INIT = 0xFFFF;  // Initial value

bool aplink_parse_byte(aplink_msg* link_msg, uint8_t byte);
uint16_t aplink_pack(uint8_t packet[], const uint8_t payload[], const uint8_t payload_len, const uint8_t msg_id);
bool aplink_unpack(const uint8_t packet[], uint8_t payload[], uint8_t& payload_len, uint8_t& msg_id);
uint16_t aplink_calc_packet_size(uint8_t payload_size);
uint16_t aplink_crc16(const uint8_t data[], size_t length);

#endif /* LIB_APLINK_APLINK_H_ */
