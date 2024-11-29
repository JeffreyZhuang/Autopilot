#include <main.h>

Vehicle vehicle;
AHRS ahrs(&vehicle);
Sensors sensors(&vehicle);
Navigation navigation(&vehicle);
DataLog data_log(&vehicle);
SWOStream swo(2000000);
uint32_t prev_loop_time;
uint32_t prev_print_time;

void setup_leds() {
  pinMode(PC1, OUTPUT);
}

void setup() {
  Serial.begin(115200);

  data_log.setup(); // BUG: Does not work when moved under i2c_begin. I haven't defined the SDIO pins, maybe pins override i2c?

  setup_leds();

  ahrs.setup();
  sensors.setup();

  digitalWrite(PC1, HIGH);
  delay(500);
  digitalWrite(PC1, LOW);
  delay(500);
}

void loop() {
  sensors.poll();
  ahrs.update();
  navigation.update();
  data_log.write();

  Serial.println((micros() - prev_loop_time) / 1000);

  if (micros() - prev_print_time > 300000) {
    
    
    // Replace with USB instead of SWO
    // swo.println("Time: " + String(millis()));
    // swo.println(String(vehicle.baro_alt) + "\t" + String(vehicle.imu_ax) + "\t" + String(vehicle.imu_ay) + 
    //             "\t" + String(vehicle.imu_gz) + "\t" + String(vehicle.compass_mx) + "\t" + 
    //             String(vehicle.compass_my) + "\t" + String(vehicle.compass_mz));
    // swo.println(vehicle.autopilot_voltage, 6);
    // swo.println(vehicle.autopilot_current, 6);
    // swo.println(vehicle.batt_voltage, 6);
    // swo.println(vehicle.batt_current, 6);
    // swo.println("dt: " + String((micros() - prev_loop_time) / 1000));
    // swo.println(String(vehicle.ahrs_roll) + "\t" + String(vehicle.ahrs_pitch) + "\t" + String(vehicle.ahrs_yaw));
    prev_print_time = micros();
  }
  
  prev_loop_time = micros();
}