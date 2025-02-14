#include "utils.h"

float clamp(float n, float min, float max)
{
    if (n > max)
    {
        n = max;
    }

    if (n < min)
    {
        n = min;
    }

    return n;
}

void lat_lon_to_meters(double lat_ref, double lon_ref, double lat, double lon, double *north, double *east) {
	double earth_radius = 6378137.0;

    double dLat = (lat - lat_ref) * M_PI / 180.0;
    double dLon = (lon - lon_ref) * M_PI / 180.0;

    double meanLat = (lat + lat_ref) / 2.0 * M_PI / 180.0;

    *north = dLat * earth_radius;
    *east = dLon * earth_radius * cos(meanLat);
}

void meters_to_lat_lon(double north, double east, double refLat, double refLon, double *lat, double *lon) {
	double earth_radius = 6378137.0;

    double refLatRad = refLat * M_PI / 180.0;

    *lat = refLat + (north / earth_radius) * (180.0 / M_PI);
    *lon = refLon + (east / (earth_radius * cos(refLatRad))) * (180.0 / M_PI);
}

uint16_t map_uint16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    // Ensure the input value is within the input range
    if (x < in_min) {
        x = in_min;
    } else if (x > in_max) {
        x = in_max;
    }

    // Calculate the scaled value
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
