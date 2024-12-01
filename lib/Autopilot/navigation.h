#include <plane.h>

/**
 * @brief Calculates the position of the plane
 */
class Navigation {
public:
    Navigation(Plane * plane);
    void update();
    void update_accelerometer();
    void read_imu();
    void read_baro();
    void read_compass();
    void read_gps();
private:
    Plane * _plane;
    uint32_t last_imu_timestamp;
    uint32_t last_baro_timestamp;
};