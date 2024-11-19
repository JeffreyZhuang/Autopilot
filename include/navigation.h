#include <vehicle.h>

class Navigation {
public:
    Navigation(Vehicle * vehicle);
    void update();
    void read_imu();
    void read_baro();
    void read_compass();
    void read_gps();
private:
    Vehicle * _vehicle;
};