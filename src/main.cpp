#include <hal.h>
#include <ahrs.h>
#include <navigation.h>
#include <plane.h>

Plane plane;
AHRS ahrs(&plane);
HAL hal(&plane);
Navigation navigation(&plane);
uint32_t prev_loop_time;
uint32_t prev_print_time;
char txBuf[500];

void setup() {
  ahrs.setup();
  hal.setup();
  hal.blink_led();
}

void loop() {
  hal.poll();

  uint32_t dt = hal.get_time_us() - prev_loop_time;
  prev_loop_time = hal.get_time_us();

  ahrs.update();
  navigation.update();
  hal.write_sd();

  strcpy(txBuf, ""); 
  sprintf(txBuf, "DT: %d\n", dt);
  hal.usb_print(txBuf);

  if (hal.get_time_us() - prev_print_time > 300000) {
    strcpy(txBuf, ""); 
    sprintf(txBuf, "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", plane.baro_alt, plane.imu_ax, plane.imu_ay, plane.imu_gz, plane.compass_mx, plane.compass_my, plane.compass_mz);
    hal.swo_print(txBuf);

    prev_print_time = hal.get_time_us();
  }
}