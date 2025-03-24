#include "autopilot_link.h"

bool Autopilot_link::parse_byte(uint8_t byte)
{
	if (byte == START_BYTE)
	{
		_in_pkt = true;
		_pkt_idx = 0;
	}

	if (_in_pkt)
	{
		// Append byte to packet
		_packet[_pkt_idx] = byte;
		_pkt_idx++;

		switch (_pkt_idx)
		{
		case 1:
			break;
		case 2:
			_payload_len = byte;
			break;
		case 3:
			_msg_id = byte;
			break;
		case 4:
			_cobs_byte = byte;
			break;
		case MAX_PACKET_LEN:
			// Reset
			_pkt_idx = 0;
			_in_pkt = false;

			break;
		default:
			if (_pkt_idx == _payload_len + HEADER_LEN)
			{
				// Parse


				// Reset
				_pkt_idx = 0;
				_in_pkt = false;
			}
			break;
		}
	}
}

void Autopilot_link::pack(uint8_t packet[], const uint8_t payload[],
						  const uint8_t payload_len, const uint8_t msg_id)
{
	uint16_t packet_index = 0;

	// Header
	packet[packet_index++] = START_BYTE;
	packet[packet_index++] = payload_len;
	packet[packet_index++] = msg_id;

	// Encode payload with COBS
	uint8_t payload_cobs[payload_len + 1];
	cobs_encode(payload_cobs, sizeof(payload_cobs), payload, payload_len);

	// Add COBS byte and encoded payload
	for (uint8_t i = 0; i < sizeof(payload_cobs); i++)
	{
		packet[packet_index++] = payload_cobs[i];
	}

	// Compute checksum excluding start byte
	uint16_t checksum = crc16(&packet[1], packet_index - 1);
	packet[packet_index++] = (checksum >> 8) & 0xFF; // High byte
	packet[packet_index++] = checksum & 0xFF; // Low byte
}

bool Autopilot_link::unpack(const uint8_t packet[], uint8_t payload[], uint8_t& payload_len, uint8_t& msg_id)
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
