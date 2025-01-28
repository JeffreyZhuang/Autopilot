#include <hal.h>

/**
 * @brief Construct a new HAL::HAL object
 *
 * @param plane
 */
HAL::HAL(Plane* plane) {
    _plane = plane;
}
