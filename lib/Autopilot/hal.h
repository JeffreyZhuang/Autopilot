#include <stdint.h>
#include <plane.h>

class HAL {
public:
    HAL(Plane * plane);

    void setup();
    void poll();
    void blink_led();
    void delay_us(uint32_t us);
    void swo_print(char * str);
    void usb_print(char * str);
    void write_sd();
    uint32_t get_time_us();
};