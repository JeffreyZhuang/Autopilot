#ifndef MODULES_STORAGE_STORAGE_H_
#define MODULES_STORAGE_STORAGE_H_

#include "lib/autopilot_link/autopilot_link.h"
#include <data_bus.h>
#include "hal.h"
#include "module.h"
#include <stdint.h>
#include <cstring>
#include <stdio.h>

struct __attribute__((packed))Storage_payload
{
	uint32_t loop_iteration;
	uint64_t time;
	float gyro[3];
	float accel[3];
	float mag_uncalib[3];
	float nav_pos[3];
	float nav_vel[3];
	float baro_alt;
	float rc_channels[4];
	double gnss_lat;
	double gnss_lon;
	bool gps_fix;
	uint8_t mode_id;
};

class Storage : public Module
{
public:
	Storage(HAL* hal, Data_bus* data_bus);

	void update() override;
	void update_background();

private:
	static constexpr int payload_size = sizeof(Storage_payload);
	static constexpr int packet_size = payload_size + 2; // Add start byte and COBS
	static constexpr int buffer_size = 200 * packet_size;
	uint8_t front_buffer[buffer_size];
	uint8_t back_buffer[buffer_size];
	bool front_buff_full = false;
	uint32_t back_buff_last_idx = 0;

	Subscriber<IMU_data> _imu_sub;
	Subscriber<Mag_data> _mag_sub;
	Subscriber<GNSS_data> _gnss_sub;
	Subscriber<Pos_est_data> _pos_est_sub;
	Subscriber<Baro_data> _baro_sub;
	Subscriber<Modes_data> _modes_sub;
	Subscriber<Time_data> _time_sub;
	Subscriber<RC_data> _rc_sub;

	IMU_data _imu_data{};
	Mag_data _mag_data{};
	GNSS_data _gnss_data{};
	Pos_est_data _pos_est_data{};
	Baro_data _baro_data{};
	Modes_data _modes_data{};
	Time_data _time_data{};
	RC_data _rc_data{};

	void write();
	void flush();
	void read();
	Storage_payload create_payload();
};

#endif /* MODULES_STORAGE_STORAGE_H_ */
