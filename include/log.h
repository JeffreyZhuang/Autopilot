#include <SWOStream.h>
#include <STM32SD.h>
#include <Arduino.h>
#include <vehicle_parameters.h>

extern SWOStream swo;

class DataLog {
public:
    DataLog(Vehicle * vehicle);
    void setup();
    void write();
private:
    Vehicle * _vehicle;
    Sd2Card card;
    SdFatFs fatFs;
    File file;
};