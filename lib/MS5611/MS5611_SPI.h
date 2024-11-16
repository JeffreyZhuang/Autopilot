#pragma once

// https://github.com/RobTillaart/MS5611_SPI

#include "Arduino.h"
#include "SPI.h"

#define MS5611_READ_OK                        0
#define MS5611_ERROR_2                        2         //  TODO ??
#define MS5611_NOT_READ                       -999

enum osr_t
{
  OSR_ULTRA_HIGH = 12,        // 10 millis
  OSR_HIGH       = 11,        //  5 millis
  OSR_STANDARD   = 10,        //  3 millis
  OSR_LOW        = 9,         //  2 millis
  OSR_ULTRA_LOW  = 8          //  1 millis    Default = backwards compatible
};

class MS5611_SPI
{
public:
  explicit MS5611_SPI(uint8_t select, SPIClass * mySPI = &SPI);
  bool     begin();
  bool     reset(uint8_t mathMode = 0);
  int      read(uint8_t bits);
  inline int read() { return read( (uint8_t) _samplingRate); };
  void     setOversampling(osr_t samplingRate);
  float    getTemperature() const;
  float    getPressure() const;
  void     setSPIspeed(uint32_t speed);
protected:
  void     convert(const uint8_t addr, uint8_t bits);
  uint32_t readADC();
  uint16_t readProm(uint8_t reg);
  void      command(const uint8_t command);
  void     initConstants(uint8_t mathMode);
  uint8_t  _samplingRate;
  int32_t  _temperature;
  int32_t  _pressure;
  int      _result;
  float    C[7];
  uint8_t  _select;
  uint32_t _SPIspeed = 1000000;
  SPIClass * _mySPI;
  SPISettings   _spi_settings;
  int state = 0;
};

