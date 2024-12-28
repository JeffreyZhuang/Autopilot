/*
 * autopilot_main.hpp
 *
 *  Created on: Dec. 6, 2024
 *      Author: jeffr
 */

#ifndef INC_AUTOPILOT_MAIN_H_
#define INC_AUTOPILOT_MAIN_H_

void SD_init();
void SD_write();

void main_loop();
void autopilot_main();

#ifdef __cplusplus
extern "C"
{
#endif

void autopilot_main_c();

#ifdef __cplusplus
}
#endif

#endif /* INC_AUTOPILOT_MAIN_H_ */
