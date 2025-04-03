#ifndef LIB_APLINK_MESSAGES_VFR_HUD_H_
#define LIB_APLINK_MESSAGES_VFR_HUD_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t VFR_HUD_MSG_ID = 10;

struct __attribute__((packed)) aplink_vfr_hud
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

inline uint16_t aplink_vfr_hud_pack(aplink_vfr_hud vfr_hud, uint8_t packet[])
{
	uint8_t buffer[sizeof(vfr_hud)];
	memcpy(&buffer, &vfr_hud, sizeof(buffer));
	return aplink_pack(packet, buffer, sizeof(buffer), VFR_HUD_MSG_ID);
}

inline bool aplink_vfr_hud_msg_decode(aplink_msg* msg, aplink_vfr_hud* vfr_hud)
{
	if (msg->payload_len == sizeof(aplink_vfr_hud))
	{
		memcpy(vfr_hud, msg->payload, msg->payload_len);
		return true;
	}
	return false;
}

#endif /* LIB_APLINK_MESSAGES_VFR_HUD_H_ */
