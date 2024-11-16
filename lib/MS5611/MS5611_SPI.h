#pragma once
//
//    FILE: MS5611_SPI.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.3.1
// PURPOSE: S5611 (SPI) Temperature & Pressure library for Arduino
//     URL: https://github.com/RobTillaart/MS5611_SPI


#include "Arduino.h"
#include "SPI.h"


//  BREAKOUT  MS5611  aka  GY63 - see datasheet
//
//  SPI    I2C
//              +--------+
//  VCC    VCC  | o      |
//  GND    GND  | o      |
//         SCL  | o      |
//  SDI    SDA  | o      |
//  CSO         | o      |
//  SDO         | o L    |   L = led
//          PS  | o    O |   O = opening  PS = protocol select
//              +--------+
//
//  PS to VCC  ==>  I2C  (GY-63 board has internal pull up, so not needed)
//  PS to GND  ==>  SPI
//  CS to VCC  ==>  0x76
//  CS to GND  ==>  0x77


#define MS5611_SPI_LIB_VERSION                (F("0.3.1 EXPERIMENTAL"))

#ifndef __SPI_CLASS__
  //  MBED must be tested before RP2040
  #if defined(ARDUINO_ARCH_MBED)
  #define __SPI_CLASS__   SPIClass
  #elif defined(ARDUINO_ARCH_RP2040)
  #define __SPI_CLASS__   SPIClassRP2040
  #else
  #define __SPI_CLASS__   SPIClass
  #endif
#endif


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
  //       HARDWARE SPI
  explicit MS5611_SPI(uint8_t select, __SPI_CLASS__ * mySPI = &SPI);
  //       SOFTWARE SPI
  explicit MS5611_SPI(uint8_t select, uint8_t dataOut, uint8_t dataIn, uint8_t clock);

  bool     begin();
 
  //       reset command + get constants
  //       mathMode = 0 (default), 1 = factor 2 fix.
  //       returns false if ROM constants == 0;
  bool     reset(uint8_t mathMode = 0);

  //  the actual reading of the sensor;
  //  returns MS5611_READ_OK upon success
  int      read(uint8_t bits);
  //  wrapper, uses the preset oversampling rate.
  inline int read() { return read( (uint8_t) _samplingRate); };

  //  sets oversampling to a value between 8 and 12
  void     setOversampling(osr_t samplingRate);


  //  temperature is in ²C
  float    getTemperature() const;

  //  pressure is in mBar
  float    getPressure() const;

  //       speed in Hz
  void     setSPIspeed(uint32_t speed);


protected:
  void     convert(const uint8_t addr, uint8_t bits);
  uint32_t readADC();
  uint16_t readProm(uint8_t reg);
  int      command(const uint8_t command);
  void     initConstants(uint8_t mathMode);

  uint8_t  _samplingRate;
  int32_t  _temperature;
  int32_t  _pressure;
  float    _pressureOffset;
  float    _temperatureOffset;
  int      _result;
  float    C[7];
  uint32_t _lastRead;
  bool     _compensation;

  uint8_t  _select;
  uint8_t  _dataIn;
  uint8_t  _dataOut;
  uint8_t  _clock;
  uint32_t _SPIspeed = 1000000;

  __SPI_CLASS__ * _mySPI;
  SPISettings   _spi_settings;

  int state = 0;
};


//  -- END OF FILE --

