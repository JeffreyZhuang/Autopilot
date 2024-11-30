#include <hal.h>
#include <log.h>
#include <ahrs.h>
#include <navigation.h>
#include <plane.h>

// If I use library, then I can FINISH THE CODE ON COMPUTER AND THEN UPLOAD TO ARDUINO LATER
// That means I can also code at school

Plane plane;
AHRS ahrs(&plane);
HAL hal(&plane);
Navigation navigation(&plane);
DataLog data_log(&plane);
SWOStream swo(2000000);
uint32_t prev_loop_time;
uint32_t prev_print_time;

void setup_leds() {
  pinMode(PC1, OUTPUT);
}

void setup() {
  Serial.begin(115200);

  data_log.setup(); // BUG: Datalog setup does not work when moved under i2c_begin. I haven't defined the SDIO pins, maybe pins override i2c?

  setup_leds();

  ahrs.setup();
  hal.setup();

  digitalWrite(PC1, HIGH);
  delay(500);
  digitalWrite(PC1, LOW);
  delay(500);
}

void loop() {
  hal.poll();
  ahrs.update();
  navigation.update();
  data_log.write();

  Serial.println((micros() - prev_loop_time) / 1000);

  if (micros() - prev_print_time > 300000) {
    // Replace with USB instead of SWO
    // swo.println("Time: " + String(millis()));
    swo.println(String(plane.baro_alt) + "\t" + String(plane.imu_ax) + "\t" + String(plane.imu_ay) + 
                "\t" + String(plane.imu_gz) + "\t" + String(plane.compass_mx) + "\t" + 
                String(plane.compass_my) + "\t" + String(plane.compass_mz));
    // swo.println(plane.autopilot_voltage, 6);
    // swo.println(plane.autopilot_current, 6);
    // swo.println(plane.batt_voltage, 6);
    // swo.println(plane.batt_current, 6);
    // swo.println("dt: " + String((micros() - prev_loop_time) / 1000));
    // swo.println(String(plane.ahrs_roll) + "\t" + String(plane.ahrs_pitch) + "\t" + String(plane.ahrs_yaw));
    prev_print_time = micros();
  }
  
  prev_loop_time = micros();
}