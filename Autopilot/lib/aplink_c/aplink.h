#ifndef APLINK_H_
#define APLINK_H_

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#define START_BYTE 0xFE

#define HEADER_LEN 3
#define FOOTER_LEN 2

#define MAX_PAYLOAD_LEN 255

#define MAX_PACKET_LEN (MAX_PAYLOAD_LEN + HEADER_LEN + FOOTER_LEN)

typedef struct aplink_msg
{
	uint16_t checksum;
	uint8_t payload_len;
	uint8_t msg_id;
	uint8_t packet[MAX_PACKET_LEN];
	uint8_t payload[MAX_PAYLOAD_LEN];
	uint8_t packet_idx; // Index in current packet for parsing
	bool start_reading;
} aplink_msg_t;

// Checksum configuration
#define CRC16_POLY 0x8005  // CRC-16-IBM polynomial
#define CRC16_INIT 0xFFFF  // Initial value

bool aplink_parse_byte(aplink_msg_t* link_msg, uint8_t byte);
uint16_t aplink_pack(uint8_t packet[], const uint8_t payload[], const uint8_t payload_len, const uint8_t msg_id);
bool aplink_unpack(const uint8_t packet[], uint8_t payload[], uint16_t payload_len);
uint16_t aplink_calc_packet_size(uint8_t payload_size);
uint16_t aplink_crc16(const uint8_t data[], size_t length);

#endif /* APLINK_H_ */
