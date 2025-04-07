#ifndef LIB_APLINK_MESSAGES_CALIB_CMD_H_
#define LIB_APLINK_MESSAGES_CALIB_CMD_H_

static constexpr uint8_t CALIB_CMD_MSG_ID = 15;

struct aplink_calib_cmd
{
	// Can it be empty?
};

DECLARE_APLINK_MESSAGE(aplink_calib_cmd, CALIB_CMD_MSG_ID);

#endif /* LIB_APLINK_MESSAGES_CALIB_CMD_H_ */
