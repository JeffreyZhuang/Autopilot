#include <SWOStream.h>
#include <STM32SD.h>
#include <Arduino.h>
#include <plane.h>

extern SWOStream swo;

class DataLog {
public:
    DataLog(Plane * plane);
    void setup();
    void write();
private:
    Plane * _plane;
    Sd2Card card;
    SdFatFs fatFs;
    File file;
};