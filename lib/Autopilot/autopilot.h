#include <plane.h>

class Autopilot {
public:
    Autopilot();
    void setup();
    void loop();
private:
    Plane plane;
};