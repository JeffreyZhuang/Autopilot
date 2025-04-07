#ifndef LIB_APLINK_MESSAGES_VFR_HUD_H_
#define LIB_APLINK_MESSAGES_VFR_HUD_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t VFR_HUD_MSG_ID = 10;

struct __attribute__((packed)) aplink_vfr_hud
{
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	int16_t alt;
	uint16_t spd;
	int16_t roll_sp; // Flight director
	int16_t pitch_sp;
	int16_t altitude_sp;
	uint16_t spd_sp;
	uint8_t system_mode;
	uint8_t flight_mode;
	uint8_t manual_mode;
	uint8_t auto_mode;
};

DECLARE_APLINK_MESSAGE(aplink_vfr_hud, VFR_HUD_MSG_ID);

#endif /* LIB_APLINK_MESSAGES_VFR_HUD_H_ */
