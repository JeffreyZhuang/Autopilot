#ifndef MODULES_NAVIGATOR_NAVIGATOR_H_
#define MODULES_NAVIGATOR_NAVIGATOR_H_

#include <data_bus.h>
#include "params.h"
#include "lib/utils/utils.h"
#include "hal.h"
#include "module.h"

struct Waypoint
{
	double lat;
	double lon;
	float alt;
};

class Navigator : public Module
{
public:
	Navigator(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<Pos_est_data> _pos_est_sub;
	Subscriber<telem_new_waypoint_s> _telem_new_waypoint_sub;

	Publisher<Navigator_data> _navigator_pub;

	Pos_est_data _pos_est_data;
	uint8_t _curr_wp_idx = 1;
	Waypoint _waypoints[50];

	void poll();
};


#endif /* MODULES_NAVIGATOR_NAVIGATOR_H_ */
