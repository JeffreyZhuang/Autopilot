#ifndef LIB_APLINK_MESSAGES_HITL_INPUT_H_
#define LIB_APLINK_MESSAGES_HITL_INPUT_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t HITL_INPUT_MSG_ID = 4;

struct aplink_hitl_input
{
	float imu_ax;
	float imu_ay;
	float imu_az;
	float imu_gx;
	float imu_gy;
	float imu_gz;
	float mag_x;
	float mag_y;
	float mag_z;
	float baro_asl;
	int32_t gps_lat;
	int32_t gps_lon;
	int16_t of_x;
	int16_t of_y;
};

uint16_t aplink_hitl_input_pack(aplink_hitl_input data, uint8_t packet[])
{
	return aplink_pack(packet, (uint8_t*)(&data), sizeof(data), HITL_INPUT_MSG_ID);
}

aplink_hitl_input aplink_hitl_input_msg_decode(aplink_msg* msg)
{
	// Safer to write each individually
	aplink_hitl_input data;
	memcpy(&data, msg->payload, msg->payload_len);
	return data;
}

#endif /* LIB_APLINK_MESSAGES_HITL_INPUT_H_ */
