#ifndef LIB_APLINK_MESSAGES_VFR_HUD_H_
#define LIB_APLINK_MESSAGES_VFR_HUD_H_

#include "aplink.h"

struct aplink_vfr_hud
{
	int16_t roll;
	int16_t pitch;
	uint16_t heading;
	int16_t altitude;
	uint16_t speed;
	int16_t roll_setpoint; // Flight director
	int16_t pitch_setpoint;
	int16_t altitude_setpoint;
	uint8_t mode_id;
};

#endif /* LIB_APLINK_MESSAGES_VFR_HUD_H_ */
