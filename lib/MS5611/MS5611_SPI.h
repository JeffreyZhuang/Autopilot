#pragma once

// https://github.com/RobTillaart/MS5611_SPI

#include "Arduino.h"
#include "SPI.h"

#define MS5611_READ_OK 0
#define MS5611_ERROR_2 2
#define MS5611_NOT_READ -999

enum osr_t
{
  OSR_ULTRA_HIGH = 12,
  OSR_HIGH       = 11,
  OSR_STANDARD   = 10,
  OSR_LOW        = 9,
  OSR_ULTRA_LOW  = 8
};

class MS5611_SPI
{
public:
  MS5611_SPI(uint8_t select, SPIClass * mySPI, uint32_t spi_speed, osr_t samplingRate);
  bool begin();
  bool reset();
  int read();
  float getTemperature() const;
  float getPressure() const;
protected:
  void convert(const uint8_t addr);
  uint32_t readADC();
  uint16_t readProm(uint8_t reg);
  void command(const uint8_t command);
  uint8_t _samplingRate;
  int32_t _temperature;
  int32_t _pressure;
  int _result;
  float C[7];
  uint8_t _select;
  SPIClass * _mySPI;
  SPISettings _spi_settings;
  int _state = 0;
};