#include <lib/aplink/aplink.h>

bool aplink_parse_byte(aplink_msg* link_msg, uint8_t byte)
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
		else if (link_msg->packet_idx == aplink_calc_packet_size(link_msg->payload_len))
		{
			link_msg->start_reading = false;
			link_msg->packet_idx = 0;

			return aplink_unpack(link_msg->packet, link_msg->payload, link_msg->payload_len, link_msg->msg_id);
		}
		else if (link_msg->packet_idx == MAX_PACKET_LEN)
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

	// Payload
	for (uint8_t i = 0; i < payload_len; i++)
	{
		packet[index++] = payload[i];
	}

	// Compute checksum excluding start byte
	uint16_t checksum = aplink_crc16(&packet[1], index - 1);
	packet[index++] = (checksum >> 8) & 0xFF; // High byte
	packet[index++] = checksum & 0xFF; // Low byte

	return index; // Return packet size
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
    uint16_t expected_checksum = aplink_crc16(&packet[1], payload_len + HEADER_LEN - 1);
    uint16_t received_checksum = (packet[payload_len + HEADER_LEN] << 8) | packet[payload_len + HEADER_LEN + 1];

    if (expected_checksum != received_checksum)
    {
        return false; // Checksum mismatch
    }

    for (uint16_t i = 0; i < payload_len; i++)
    {
    	payload[i] = packet[i + HEADER_LEN];
    }

    return true;
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
