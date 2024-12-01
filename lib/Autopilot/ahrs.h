#include <MadgwickAHRS.h>
#include <plane.h>
#include <hal.h>

class AHRS {
public:
    AHRS(Plane * plane, HAL * hal);
    void setup();
    void update();
private:
    void update_imu();
    void update_full();

    Plane * _plane;
    HAL * _hal;
    Madgwick filter;
    uint32_t last_imu_timestamp;
    uint32_t last_compass_timestamp;
    uint32_t prev_loop_time;
};