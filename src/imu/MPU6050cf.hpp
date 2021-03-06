#pragma once

// Based on code from http://www.pitt.edu/~mpd41/Angle.ino

#include "IMU.hpp"

#ifdef __AVR_ATmega328P__
#include <Wire.h>
#else 
#include <i2c_t3.h>
#endif

class MPU6050cf : public IMU {

public:
  MPU6050cf( int addr = 0x68 );
  void init( int16_t sample_interval_us );
  void calibrate();
  bool calibrateDone();
  bool lockChannel( bool lock );
  bool sampleAvailable( void );
  void requestSample( void );
  bool sampleReady( void );
  void readSample( void );
  void computeAngles( void );
  float angleX( void );
  float angleY( void );
  float angleZ( void );
private:
  int address;
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
  uint32_t timer;    
  float dt;
};
