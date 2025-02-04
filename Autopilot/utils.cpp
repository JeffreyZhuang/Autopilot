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
    double dLat = (lat - lat_ref) * M_PI / 180.0;
    double dLon = (lon - lon_ref) * M_PI / 180.0;

    double meanLat = (lat + lat_ref) / 2.0 * M_PI / 180.0;

    *north = dLat * EARTH_RADIUS;
    *east = dLon * EARTH_RADIUS * cos(meanLat);
}

void meters_to_lat_lon(double north, double east, double refLat, double refLon, double *lat, double *lon) {
    // Convert reference latitude to radians
    double refLatRad = refLat * M_PI / 180.0;

    // Compute new latitude and longitude
    *lat = refLat + (north / EARTH_RADIUS) * (180.0 / M_PI);
    *lon = refLon + (east / (EARTH_RADIUS * cos(refLatRad))) * (180.0 / M_PI);
}
