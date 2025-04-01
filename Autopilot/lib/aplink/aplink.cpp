#include <lib/aplink/aplink.h>

bool aplink_parse_byte(Autopilot_link_msg* link_msg, uint8_t byte)
{
	if (byte == START_BYTE)
	{
		link_msg->start_reading = true;
	}

	if (link_msg->start_reading)
	{
		if (link_msg->packet_idx == 1)
		{
			link_msg->payload_len = byte;
		}
		else if (link_msg->packet_idx == 2)
		{
			link_msg->msg_id = byte;
		}
		else if (link_msg->packet_idx == calc_packet_size(link_msg->payload_len))
		{
			link_msg->start_reading = false;
			link_msg->packet_idx = 0;

			// Parse
			if (unpack(_packet, payload, payload_len, msg_id))
			{
				memcpy(latest_packet, _packet, calc_packet_size(_payload_len));
				return true;
			}
		}
		else if (link_msg->packet_idx == calc_packet_size(link_msg->payload_len))
		{

		}
		else if (_pkt_idx == MAX_PACKET_LEN)
		{
			link_msg->start_reading = false;
			link_msg->packet_idx = 0;
		}

		link_msg->packet_idx++;
	}

	return false;
}

uint16_t aplink_pack(uint8_t packet[], const uint8_t payload[], const uint8_t payload_len, const uint8_t msg_id)
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

bool aplink_unpack(const uint8_t packet[], uint8_t payload[], uint8_t& payload_len, uint8_t& msg_id)
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

uint16_t aplink_calc_packet_size(uint8_t payload_size)
{
	return payload_size + HEADER_LEN + FOOTER_LEN;
}

uint16_t aplink_crc16(const uint8_t data[], size_t length)
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
