#ifndef TELEM_H_
#define TELEM_H_

#include <lib/aplink/aplink.h>
#include "lib/utils/utils.h"
#include "hal.h"
#include "modes.h"
#include "constants.h"
#include "module.h"
#include <cstdio>
#include <cstring>

static constexpr uint16_t MAX_BYTE_RATE = 1500; // Bytes per sec

class Telem : public Module
{
public:
	Telem(HAL* hal, Data_bus* data_bus);

	void update();

private:
	uint64_t _last_tlm_transmit_time = 0; // Time of last telemetry transmission
	uint16_t _bytes_since_last_tlm_transmit = 0; // Total bytes sent since last telemetry transmission

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

	Publisher<Telem_data> _telem_pub;
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

	aplink_msg telem_msg;
	aplink_msg usb_msg;

	void read_telem();
	void read_usb();
	void transmit_usb();
	void transmit_packet(uint8_t packet[], uint16_t size);
	void transmit_telem(); // Transmit telemetry packet
	bool parse_packet();
	void ack();
	uint8_t get_current_state();
	Telem_payload create_telem_payload();
};

#endif /* TELEM_H_ */
