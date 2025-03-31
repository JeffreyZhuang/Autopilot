#ifndef LIB_APLINK_MESSAGES_NAV_DISPLAY_H_
#define LIB_APLINK_MESSAGES_NAV_DISPLAY_H_

#include "aplink.h"

struct aplink_nav_display
{
	float pos_est_north;
	float pos_est_east;
	uint8_t wp_idx;
};

#endif /* LIB_APLINK_MESSAGES_NAV_DISPLAY_H_ */
