#ifndef MODULES_USB_COMM_USB_COMM_H_
#define MODULES_USB_COMM_USB_COMM_H_

#include "lib/aplink/aplink.h"
#include "lib/aplink/aplink_messages.h"
#include "lib/utils/utils.h"
#include "module.h"

class USBComm : Module
{
public:
	USBComm(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<GNSS_data> _gnss_sub;
	Subscriber<AHRS_data> _ahrs_sub;
	Subscriber<Pos_est_data> _pos_est_sub;
	Subscriber<Modes_data> _modes_sub;
	Subscriber<L1_data> _l1_sub;
	Subscriber<Navigator_data> _navigator_sub;
	Subscriber<Power_data> _power_sub;
	Subscriber<TECS_data> _tecs_sub;
	Subscriber<Ctrl_cmd_data> _ctrl_cmd_sub;
	Subscriber<Baro_data> _baro_sub;
	Subscriber<HITL_output_data> _hitl_output_sub;
	Subscriber<Telem_data> _telem_sub;

	Publisher<HITL_data> _hitl_pub;

	Ctrl_cmd_data _ctrl_cmd_data;
	AHRS_data _ahrs_data;
	GNSS_data _gnss_data;
	Telem_data _telem_data;
	Pos_est_data _pos_est_data;
	Modes_data _modes_data;
	L1_data _l1_data;
	Navigator_data _navigator_data;
	Power_data _power_data;
	TECS_data _tecs_data;
	HITL_data _hitl_data;
	Baro_data _baro_data;

	aplink_msg msg;

	void read();
	void transmit();
};

#endif /* MODULES_USB_COMM_USB_COMM_H_ */
