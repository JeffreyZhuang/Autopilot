/*
 * request_waypoint.h
 *
 *  Created on: Apr. 6, 2025
 *      Author: jeffr
 */

#ifndef LIB_APLINK_MESSAGES_REQ_WAYPOINT_H_
#define LIB_APLINK_MESSAGES_REQ_WAYPOINT_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t REQ_WAYPOINT_MSG_ID = 10;

struct aplink_req_waypoint
{
	uint8_t index;
};

DECLARE_APLINK_MESSAGE(aplink_req_waypoint, REQ_WAYPOINT_MSG_ID);

#endif /* LIB_APLINK_MESSAGES_REQ_WAYPOINT_H_ */
