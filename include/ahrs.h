#include <Arduino.h>
#include <MadgwickAHRS.h>
#include <vehicle_parameters.h>

class AHRS {
public:
    AHRS(Vehicle * vehicle);
    void setup();
    void update();
private:
    Vehicle * _vehicle;
    Madgwick filter;
};