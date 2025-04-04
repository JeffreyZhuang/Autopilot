#ifndef LIB_APLINK_MESSAGES_POWER_H_
#define LIB_APLINK_MESSAGES_POWER_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t POWER_MSG_ID = 11;

struct __attribute__((packed)) aplink_power
{
	uint16_t battery_voltage;
	uint16_t battery_current;
	uint16_t battery_used;
	uint16_t autopilot_current;
};

DECLARE_APLINK_MESSAGE(aplink_power, POWER_MSG_ID);

#endif /* LIB_APLINK_MESSAGES_POWER_H_ */
