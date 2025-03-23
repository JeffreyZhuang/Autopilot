#ifndef MODULES_UTILS_H_
#define MODULES_UTILS_H_

#include <cmath>
#include <stdint.h>

float clamp(float n, float min, float max);
double lat_lon_to_distance(double lat_ref, double lon_ref, double lat, double lon);
void lat_lon_to_meters(double lat_ref, double lon_ref, double lat, double lon, double *north, double *east);
void meters_to_lat_lon(double north, double east, double refLat, double refLon, double *lat, double *lon);
float map(float x, float in_min, float in_max, float out_min, float out_max);
float lerp(float x0, float y0, float x1, float y1, float x);
float wrap_pi(float angle);
float max(float a, float b);

#endif /* MODULES_UTILS_H_ */
