#include <plane.h>

class Navigation {
public:
    Navigation(Plane * plane);
    void update();
    void read_imu();
    void read_baro();
    void read_compass();
    void read_gps();
private:
    Plane * _plane;
};