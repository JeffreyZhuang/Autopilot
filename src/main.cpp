#include <Arduino.h>
#include <autopilot.h>

Autopilot autopilot;

void setup() {
  autopilot.setup();
}

void loop() {
  autopilot.loop();
}