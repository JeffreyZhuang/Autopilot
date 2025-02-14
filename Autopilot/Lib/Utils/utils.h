/*
 * utils.h
 *
 *  Created on: Feb. 2, 2025
 *      Author: jeffr
 */

#ifndef MODULES_UTILS_H_
#define MODULES_UTILS_H_

#include <cmath>
#include <stdint.h>

float clamp(float n, float min, float max);
void lat_lon_to_meters(double lat_ref, double lon_ref, double lat, double lon, double *north, double *east);
void meters_to_lat_lon(double north, double east, double refLat, double refLon, double *lat, double *lon);
uint16_t map_uint16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

#endif /* MODULES_UTILS_H_ */
