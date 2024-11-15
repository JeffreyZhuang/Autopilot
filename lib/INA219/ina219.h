#ifndef INA219_H_
#define INA219_H_

#include <Wire.h>

class INA219 {
public:
    INA219(TwoWire * i2c_bus, uint8_t i2c_address, float shunt_resistance);
    float read_voltage();
    float read_current();
private:
    uint16_t read_register(uint8_t reg);
    TwoWire * _i2c_bus;
    uint8_t _i2c_address;
    float _shunt_resistance;
};

#endif /* INA219_H_ */