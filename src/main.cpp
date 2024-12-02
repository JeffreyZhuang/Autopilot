#include <autopilot.h>
#include <plane.h>
#include <hal_arduino.h>

Plane plane;
HAL_Arduino hal_arduino(&plane);
Autopilot autopilot(&hal_arduino, &plane);

/**
 * @brief Setup
 * 
 */
void setup() {
  autopilot.setup();
}

/**
 * @brief Loop
 * 
 */
void loop() {
  autopilot.loop();
}