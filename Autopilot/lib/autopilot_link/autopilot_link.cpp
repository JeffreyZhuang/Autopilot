#include "autopilot_link.h"

bool Autopilot_link::parse_byte(uint8_t byte, uint8_t payload[], uint8_t& msg_id)
{
	if (byte == START_BYTE)
	{
		_in_pkt = true;
		_pkt_idx = 0;
	}

	if (_in_pkt)
	{
		// Append byte to packet
		_packet[_pkt_idx++] = byte;

		if (_pkt_idx == 2)
		{
			_payload_len = byte;
		}
		else if (_pkt_idx == _payload_len + HEADER_LEN)
		{
			_in_pkt = false;

			// Parse
			uint8_t payload_len_result;
			return unpack(_packet, payload, payload_len_result, msg_id);
		}
		else if (_pkt_idx == MAX_PACKET_LEN)
		{
			_in_pkt = false;
		}
	}

	return false;
}

void Autopilot_link::pack(uint8_t packet[], const uint8_t payload[],
						  const uint8_t payload_len, const uint8_t msg_id)
{
	uint16_t index = 0;

	// Header
	packet[index++] = START_BYTE;
	packet[index++] = payload_len;
	packet[index++] = msg_id;

	// Encode payload with COBS
	uint8_t payload_cobs[payload_len + 1];
	cobs_encode(payload_cobs, sizeof(payload_cobs), payload, payload_len);

	// Add COBS byte and encoded payload
	for (uint8_t i = 0; i < sizeof(payload_cobs); i++)
	{
		packet[index++] = payload_cobs[i];
	}

	// Compute checksum excluding start byte
	uint16_t checksum = crc16(&packet[1], index - 1);
	packet[index++] = (checksum >> 8) & 0xFF; // High byte
	packet[index++] = checksum & 0xFF; // Low byte
}

bool Autopilot_link::unpack(const uint8_t packet[], uint8_t payload[], uint8_t& payload_len,
							uint8_t& msg_id)
{
    // Validate start byte
    if (packet[0] != START_BYTE)
    {
        return false; // Invalid packet
    }

    payload_len = packet[1];
    msg_id = packet[2];

    // Compute expected checksum
    uint16_t expected_checksum = crc16(&packet[1], payload_len + HEADER_LEN - 1);
    uint16_t received_checksum = (packet[payload_len + HEADER_LEN] << 8) | packet[payload_len + HEADER_LEN + 1];

    if (expected_checksum != received_checksum)
    {
        return false; // Checksum mismatch
    }

    // Decode payload with COBS
    cobs_decode_result res = cobs_decode(payload, payload_len, &packet[HEADER_LEN], payload_len + 1);

    return res.status == COBS_DECODE_OK;
}

uint16_t Autopilot_link::calc_packet_size(uint8_t payload_size)
{
	return payload_size + HEADER_LEN + FOOTER_LEN;
}

uint16_t Autopilot_link::crc16(const uint8_t data[], size_t length)
{
	uint16_t crc = CRC16_INIT;

	for (size_t i = 0; i < length; i++)
	{
		crc ^= (uint16_t)data[i] << 8; // XOR with input byte

		for (uint8_t j = 0; j < 8; j++)
		{
			if (crc & 0x8000)
			{
				crc = (crc << 1) ^ CRC16_POLY;
			}
			else
			{
				crc <<= 1;
			}
		}
	}

	return crc;
}
