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

void lat_lon_to_meters(double lat, double lon, double refLat, double refLon, double *north, double *east) {
    // Convert degrees to radians
    double latRad = lat * M_PI / 180.0;
    double lonRad = lon * M_PI / 180.0;
    double refLatRad = refLat * M_PI / 180.0;
    double refLonRad = refLon * M_PI / 180.0;

    // Calculate differences
    double dLat = latRad - refLatRad;
    double dLon = lonRad - refLonRad;

    // Approximate conversion (assuming small distances)
    *north = dLat * EARTH_RADIUS; // North displacement (meters)
    *east = dLon * EARTH_RADIUS * cos(refLatRad); // East displacement (meters)
}

void meters_to_lat_lon(double north, double east, double refLat, double refLon, double *lat, double *lon) {
    // Convert reference latitude to radians
    double refLatRad = refLat * M_PI / 180.0;

    // Compute new latitude and longitude
    *lat = refLat + (north / EARTH_RADIUS) * (180.0 / M_PI);
    *lon = refLon + (east / (EARTH_RADIUS * cos(refLatRad))) * (180.0 / M_PI);
}
