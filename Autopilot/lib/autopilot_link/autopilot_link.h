#ifndef LIB_AUTOPILOT_LINK_AUTOPILOT_LINK_H_
#define LIB_AUTOPILOT_LINK_AUTOPILOT_LINK_H_

#include <stdio.h>
#include <string.h>

struct __attribute__((packed)) Telem_payload
{
	int16_t roll;
	int16_t pitch;
	uint16_t yaw;
	int16_t alt;
	uint16_t spd;
	int16_t alt_setpoint;
	int32_t lat;
	int32_t lon;
	float nav_north;
	float nav_east;
	uint8_t mode_id;
	uint8_t wp_idx;
	uint16_t cell_voltage;
	uint16_t battery_current;
	uint16_t battery_used;
	uint16_t autopilot_current;
	uint8_t gps_sats;
	bool gps_fix;
	uint8_t aileron;
	uint8_t elevator;
	uint8_t throttle;
};

struct __attribute__((packed)) Waypoint_payload
{
	uint8_t waypoint_index;
	uint8_t total_waypoints;
	int32_t lat;
	int32_t lon;
	int16_t alt;
};

struct __attribute__((packed)) Params_payload
{
	// Performance
	struct __attribute__((packed))
	{
		float throttle_cruise; // Steady-state cruise throttle
	} perf;

	// Attitude control
	struct __attribute__((packed))
	{
		float ptch_kp;
		float ptch_ki;
		float roll_kp;
		float roll_ki;
	} att_ctrl;

	// Takeoff
	struct __attribute__((packed))
	{
		float alt; // Altitude to detect when takeoff is complete meters
		float ptch; // Pitch during takeoff
		float roll_lim; // Roll limit during takeoff
	} takeoff;

	// Landing
	struct __attribute__((packed))
	{
		float flare_alt; // Altitude to start flare
		float flare_sink_rate; // Target sink rate for flare, meters per second
	} landing;

	// TECS
	struct __attribute__((packed))
	{
		float aspd_cruise; // Meters per second
		float aspd_land; // Landing airspeed
		float min_aspd_mps; // Minimum airspeed meters per second
		float max_aspd_mps;
		float total_energy_kp;
		float total_energy_ki;
		float energy_balance_kp;
		float energy_balance_ki;
		float ptch_lim_deg; // Maximum pitch angle
	} tecs;

	// L1 Controller
	struct __attribute__((packed))
	{
		float period;
		float roll_lim; // Maximum roll angle in either direction
	} l1_ctrl;

	// Navigator
	struct __attribute__((packed))
	{
		float min_dist_wp; // Distance in meters from waypoint until switching to next
	} navigator;

	// Optical flow
	struct __attribute__((packed))
	{
		bool enable_of; // Enable optical flow
		int16_t of_min; // Maximum optical flow value in pixels per sec
		int16_t of_max;
	} sensors;

	// Mixer
	struct __attribute__((packed))
	{
		uint16_t pwm_max_ele;
		uint16_t pwm_max_rud;
		uint16_t pwm_max_thr;
		uint16_t pwm_max_aux1;
		uint16_t pwm_max_aux2;
		uint16_t pwm_max_aux3;
		uint16_t pwm_min_ele;
		uint16_t pwm_min_rud;
		uint16_t pwm_min_thr;
		uint16_t pwm_min_aux1;
		uint16_t pwm_min_aux2;
		uint16_t pwm_min_aux3;
		bool pwm_rev_ele;
		bool pwm_rev_rud;
		bool pwm_rev_thr;
		bool pwm_rev_aux1;
		bool pwm_rev_aux2;
		bool pwm_rev_aux3;
	} mixer;

	// RC Transmitter Input
	struct __attribute__((packed))
	{
		uint16_t max_duty; // RC Transmitter stick max duty cycle microseconds
		uint16_t min_duty; // Make sure values are INSIDE the range of radio, NEVER outside
	} rc_input;

	// AHRS
	struct __attribute__((packed))
	{
		float beta_gain; // Madgwick filter gain
		float mag_decl; // Declination in degrees, determined from online calculator
		float acc_max; // Max acceleration in units of g to enable sensor fusion
		float hard_iron_x;
		float hard_iron_y;
		float hard_iron_z;
		float soft_iron_xx;
		float soft_iron_xy;
		float soft_iron_xz;
		float soft_iron_yx;
		float soft_iron_yy;
		float soft_iron_yz;
		float soft_iron_zx;
		float soft_iron_zy;
		float soft_iron_zz;
	} ahrs;

	// Position Estimator
	struct __attribute__((packed))
	{
		float baro_var; // Variance
		float gnss_var;
	} pos_estimator;

	// HITL
	struct __attribute__((packed))
	{
		bool enable;
	} hitl;
};

struct __attribute__((packed)) Time_payload
{
	uint64_t us_since_epoch;
};

// Message identifiers
static constexpr uint8_t TELEM_MSG_ID = 1;
static constexpr uint8_t WPT_MSG_ID = 2;
static constexpr uint8_t PARAMS_MSG_ID = 3;
static constexpr uint8_t HITL_MSG_ID = 4;

static constexpr uint8_t START_BYTE = 0xFE;

static constexpr uint8_t HEADER_LEN = 4;
static constexpr uint8_t FOOTER_LEN = 2;

static constexpr uint8_t MAX_PAYLOAD_LEN = 255;
static constexpr uint16_t MAX_PACKET_LEN = MAX_PAYLOAD_LEN + HEADER_LEN + FOOTER_LEN;

static constexpr uint16_t CRC16_POLY = 0x8005;  // CRC-16-IBM polynomial
static constexpr uint16_t CRC16_INIT = 0xFFFF;  // Initial value

struct aplink_message_t
{
	uint16_t checksum;
	uint8_t len;
	uint8_t msg_id;
	uint8_t payload[MAX_PAYLOAD_LEN];
};

class Autopilot_link
{
public:
	uint8_t latest_packet[MAX_PACKET_LEN];
	uint8_t latest_packet_len = 0;

	bool parse_byte(uint8_t byte, uint8_t payload[], uint8_t& payload_len, uint8_t& msg_id);
	void pack(uint8_t packet[], const uint8_t payload[],
			  const uint8_t payload_len, const uint8_t msg_id);
	bool unpack(const uint8_t packet[], uint8_t payload[], uint8_t& payload_len, uint8_t& msg_id);
	uint16_t calc_packet_size(uint8_t payload_size);

private:
	uint8_t _packet[MAX_PACKET_LEN];
	uint16_t _pkt_idx;
	bool _in_pkt = false;
	uint8_t _payload_len;

	uint16_t crc16(const uint8_t data[], size_t length);
};

#endif /* LIB_AUTOPILOT_LINK_AUTOPILOT_LINK_H_ */
