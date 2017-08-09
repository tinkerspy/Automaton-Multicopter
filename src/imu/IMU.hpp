#pragma once

#include <Arduino.h>

#define DEGREE_CONVERT 57.2957786

class IMU {

public:
  virtual void init( int16_t sample_interval_us ) = 0; // Kost dit een pointer per method???
  virtual void calibrate() = 0;
  virtual bool calibrateDone() = 0;
  virtual bool lockChannel( bool lock ) = 0;
  virtual bool sampleAvailable( void ) = 0;
  virtual void requestSample( void ) = 0;
  virtual bool sampleReady( void ) = 0;
  virtual void readSample( void ) = 0;
  virtual void computeAngles( void ) = 0;
  virtual float angleX( void ) = 0;
  virtual float angleY( void ) = 0;
  virtual float angleZ( void ) = 0;

protected:
  float compAngleX, compAngleY;
};

