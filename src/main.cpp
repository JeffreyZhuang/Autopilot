#include <autopilot.h>
#include <plane.h>
#include <hal_arduino.h>

Plane plane;
HAL_Arduino hal_arduino(&plane);
Autopilot autopilot(&hal_arduino, &plane);

void setup() {
  autopilot.setup();
}

void loop() {
  autopilot.loop();
}