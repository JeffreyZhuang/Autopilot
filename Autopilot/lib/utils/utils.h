#ifndef MODULES_UTILS_H_
#define MODULES_UTILS_H_

#include <cmath>
#include <stdint.h>
#include "lib/data_bus/modes.h"

extern "C"
{
#include "lib/aplink_c/aplink.h"
#include "lib/aplink_c/aplink_messages.h"
}

float clamp(float n, float min, float max);
double lat_lon_to_distance(double lat_ref, double lon_ref, double lat, double lon);
void lat_lon_to_meters(double lat_ref, double lon_ref, double lat, double lon, double *north, double *east);
void meters_to_lat_lon(double north, double east, double refLat, double refLon, double *lat, double *lon);
float map(float x, float in_min, float in_max, float out_min, float out_max);
float lerp(float x0, float y0, float x1, float y1, float x);
float wrap_pi(float angle);
float distance(float n1, float e1, float n2, float e2);
uint8_t get_mode_id(System_mode system_mode, Flight_mode flight_mode,
					Auto_mode auto_mode, Manual_mode manual_mode);

#endif /* MODULES_UTILS_H_ */
