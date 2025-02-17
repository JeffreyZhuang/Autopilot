#ifndef MODULES_STORAGE_STORAGE_H_
#define MODULES_STORAGE_STORAGE_H_

#include "plane.h"
#include "hal.h"
#include "Lib/COBS/cobs.h"
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
	float rc_rudder;
	float rc_elevator;
	float rc_throttle;
	double gnss_lat;
	double gnss_lon;
	bool gps_fix;
	uint8_t mode_id;
};

class Storage
{
public:
	Storage(Plane* plane, HAL* hal);

	void write();
	void flush();
	void read();

private:
	Plane* _plane;
	HAL* _hal;

	static constexpr int payload_size = sizeof(Storage_payload);
	static constexpr int packet_size = payload_size + 2; // Add start byte and COBS
	static constexpr int buffer_size = 200 * packet_size;
	uint8_t front_buffer[buffer_size];
	uint8_t back_buffer[buffer_size];
	bool front_buff_full = false;
	uint32_t back_buff_last_idx = 0;

};

#endif /* MODULES_STORAGE_STORAGE_H_ */
