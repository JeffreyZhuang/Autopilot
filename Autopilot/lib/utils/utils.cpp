#include "lib/utils/utils.h"

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

double lat_lon_to_distance(double lat_ref, double lon_ref, double lat, double lon) {
    double earth_radius = 6378137.0; // Earth's radius in meters

    double dLat = (lat - lat_ref) * M_PI / 180.0;
    double dLon = (lon - lon_ref) * M_PI / 180.0;

    double lat1 = lat_ref * M_PI / 180.0;
    double lat2 = lat * M_PI / 180.0;

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return earth_radius * c; // Great-circle distance in meters
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

float map(float x, float in_min, float in_max, float out_min, float out_max) {
    // Ensure the input value is within the input range
    if (x < in_min) {
        x = in_min;
    } else if (x > in_max) {
        x = in_max;
    }

    // Calculate the scaled value
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float lerp(float x0, float y0, float x1, float y1, float x) {
    // Check if x0 and x1 are the same to avoid division by zero
    if (x0 == x1) {
        return 0.0f;
    }

    // Calculate the interpolated value y
    float y = y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    return y;
}

// Normalize angles from [0, 360] to [-180, 180]
float wrap_pi(float angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

float max(float a, float b) {
    return (a > b) ? a : b;
}

float min(float a, float b) {
    return (a < b) ? a : b;
}
