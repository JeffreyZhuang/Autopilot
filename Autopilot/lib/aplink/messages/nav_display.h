#ifndef LIB_APLINK_MESSAGES_NAV_DISPLAY_H_
#define LIB_APLINK_MESSAGES_NAV_DISPLAY_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t NAV_DISPLAY_MSG_ID = 4;

struct __attribute__((packed)) aplink_nav_display
{
	float pos_est_north;
	float pos_est_east;
	uint8_t wp_idx;
};

uint16_t aplink_nav_display_pack(aplink_nav_display nav_display, uint8_t packet[])
{
	uint8_t buffer[sizeof(nav_display)];
	memcpy(&buffer, &nav_display, sizeof(buffer));
	return aplink_pack(packet, buffer, sizeof(buffer), NAV_DISPLAY_MSG_ID);
}


#endif /* LIB_APLINK_MESSAGES_NAV_DISPLAY_H_ */
