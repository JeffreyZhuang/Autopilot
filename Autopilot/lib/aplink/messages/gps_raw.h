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

#endif /* LIB_APLINK_MESSAGES_GPS_RAW_H_ */
