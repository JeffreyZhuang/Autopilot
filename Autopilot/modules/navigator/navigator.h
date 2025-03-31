#ifndef MODULES_NAVIGATOR_NAVIGATOR_H_
#define MODULES_NAVIGATOR_NAVIGATOR_H_

#include <data_bus.h>
#include "lib/utils/utils.h"
#include "hal.h"
#include "module.h"

class Navigator : public Module
{
public:
	Navigator(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<Pos_est_data> _pos_est_sub;
	Subscriber<Telem_data> _telem_sub;

	Publisher<Navigator_data> _navigator_pub;

	uint8_t _curr_wp_idx = 1;
};


#endif /* MODULES_NAVIGATOR_NAVIGATOR_H_ */
