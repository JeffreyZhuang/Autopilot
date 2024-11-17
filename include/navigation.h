#include <vehicle.h>

class Navigation {
public:
    Navigation(Vehicle * vehicle);
    void update();
private:
    Vehicle * _vehicle;
};