#ifndef LIB_APLINK_MESSAGES_RC_INPUT_H_
#define LIB_APLINK_MESSAGES_RC_INPUT_H_

struct aplink_rc_input
{
	uint8_t rudder; // Percentage
	uint8_t elevator;
	uint8_t throttle;
};

#endif /* LIB_APLINK_MESSAGES_RC_INPUT_H_ */
