// Poll sensors. This is where all sensor data is stored to be used by other classes (navigation, AHRS)
// Store vehicle orientation and sensor data in a global state?

// Don't include arduino in main. Only include in the drivers. That way, code is less hardware dependent.
// Put all your data variables into a struct and pass a pointer to that struct

#include <navigation.h>

Navigation::Navigation(Plane * plane) {
    _plane = plane;
}

void Navigation::update() {

}