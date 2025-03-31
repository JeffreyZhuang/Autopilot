#ifndef LIB_APLINK_MESSAGES_PARAM_SET_H_
#define LIB_APLINK_MESSAGES_PARAM_SET_H_

#include <stdio.h>

struct __attribute__((packed)) aplink_param_set
{
	char param_id[16];
	union {
		int32_t i32_val;
		float float_val;
	} value;
};

#endif /* LIB_APLINK_MESSAGES_PARAM_SET_H_ */
