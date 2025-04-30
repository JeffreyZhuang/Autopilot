#ifndef MODULES_USB_COMM_USB_COMM_H_
#define MODULES_USB_COMM_USB_COMM_H_

#include "lib/utils/utils.h"
#include "params.h"
#include "module.h"

extern "C"
{
#include "lib/aplink_c/aplink.h"
#include "lib/aplink_c/aplink_messages.h"
}

class USBComm : Module
{
public:
	USBComm(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<GNSS_data> _gnss_sub;
	Subscriber<AHRS_data> _ahrs_sub;
	Subscriber<local_position_s> _local_pos_sub;
	Subscriber<Modes_data> _modes_sub;
	Subscriber<position_control_s> _position_control_sub;
	Subscriber<waypoint_s> _waypoint_sub;
	Subscriber<Power_data> _power_sub;
	Subscriber<Ctrl_cmd_data> _ctrl_cmd_sub;
	Subscriber<Baro_data> _baro_sub;
	Subscriber<HITL_output_data> _hitl_output_sub;

	Publisher<hitl_sensors_s> _hitl_sensors_pub;

	Ctrl_cmd_data _ctrl_cmd_data;
	AHRS_data _ahrs_data;
	GNSS_data _gnss_data;
	local_position_s _local_pos;
	Modes_data _modes_data{};
	position_control_s _position_control{};
	Power_data _power_data{};
	hitl_sensors_s _hitl_sensors;
	Baro_data _baro_data;

	aplink_msg msg;

	bool read_usb();
	void read_hitl();
	void transmit_hitl();
	void transmit_debug();
};

#endif /* MODULES_USB_COMM_USB_COMM_H_ */
