#ifndef MODULES_NAVIGATOR_NAVIGATOR_H_
#define MODULES_NAVIGATOR_NAVIGATOR_H_

#include "lib/data_bus/data_bus.h"
#include "lib/mission/mission.h"
#include "lib/parameters/params.h"
#include "lib/utils/utils.h"
#include "hal.h"
#include "module.h"

class Navigator : public Module
{
public:
	Navigator(HAL* hal, DataBus* data_bus);

	void update() override;

private:
	Subscriber<local_position_s> _local_pos_sub;
	Publisher<waypoint_s> _waypoint_pub;

	local_position_s _local_pos;
	uint8_t _curr_wp_idx = 1;

	void poll_data_bus();
};


#endif /* MODULES_NAVIGATOR_NAVIGATOR_H_ */
