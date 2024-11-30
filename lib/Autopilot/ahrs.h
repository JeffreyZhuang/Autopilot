#include <MadgwickAHRS.h> // Don't use the library because it uses arduino. Actually it doesn't use arduino so should be fine.
#include <plane.h>

class AHRS {
public:
    AHRS(Plane * plane);
    void setup();
    void update();
private:
    Plane * _plane;
    Madgwick filter;
};