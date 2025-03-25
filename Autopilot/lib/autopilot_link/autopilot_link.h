#ifndef LIB_AUTOPILOT_LINK_AUTOPILOT_LINK_H_
#define LIB_AUTOPILOT_LINK_AUTOPILOT_LINK_H_

#include "lib/cobs/cobs.h"

static constexpr uint8_t START_BYTE = 0x00;

static constexpr uint8_t HEADER_LEN = 4;
static constexpr uint8_t FOOTER_LEN = 2;

static constexpr uint8_t MAX_PAYLOAD_LEN = 255;
static constexpr uint16_t MAX_PACKET_LEN = MAX_PAYLOAD_LEN + HEADER_LEN + FOOTER_LEN;

static constexpr uint16_t CRC16_POLY = 0x8005;  // CRC-16-IBM polynomial
static constexpr uint16_t CRC16_INIT = 0xFFFF;  // Initial value

class Autopilot_link
{
public:
	bool parse_byte(uint8_t byte, uint8_t payload[], uint8_t& msg_id);
	void pack(uint8_t packet[], const uint8_t payload[],
			  const uint8_t payload_len, const uint8_t msg_id);
	bool unpack(const uint8_t packet[], uint8_t payload[], uint8_t& payload_len, uint8_t& msg_id);
	uint16_t calc_packet_size(uint8_t payload_size);

private:
	uint8_t _packet[MAX_PACKET_LEN];
	uint16_t _pkt_idx;
	bool _in_pkt = false;
	uint8_t _payload_len;

	uint16_t crc16(const uint8_t data[], size_t length);
};

#endif /* LIB_AUTOPILOT_LINK_AUTOPILOT_LINK_H_ */
