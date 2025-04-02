#ifndef LIB_APLINK_MESSAGES_GPS_RAW_H_
#define LIB_APLINK_MESSAGES_GPS_RAW_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t GPS_RAW_MSG_ID = 0;

struct aplink_gps_raw
{
	int32_t lat;
	int32_t lon;
	uint8_t sats;
	bool fix;
};

uint16_t aplink_gps_raw_pack(aplink_gps_raw gps_raw, uint8_t packet[])
{
	uint8_t buffer[sizeof(gps_raw)];
	memcpy(&buffer, &gps_raw, sizeof(buffer));
	return aplink_pack(packet, buffer, sizeof(buffer), GPS_RAW_MSG_ID);
}

#endif /* LIB_APLINK_MESSAGES_GPS_RAW_H_ */
