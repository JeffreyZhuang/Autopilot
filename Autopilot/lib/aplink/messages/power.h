#ifndef LIB_APLINK_MESSAGES_POWER_H_
#define LIB_APLINK_MESSAGES_POWER_H_

#include "aplink.h"

struct __attribute__((packed)) aplink_power
{
	uint16_t battery_voltage;
	uint16_t battery_current;
	uint16_t battery_used;
	uint16_t autopilot_current;
};

#endif /* LIB_APLINK_MESSAGES_POWER_H_ */
