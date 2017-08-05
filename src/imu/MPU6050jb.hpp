#pragma once

#include "IMU.hpp"

#ifdef __AVR_ATmega328P__
#include <Wire.h>
#else 
#include <i2c_t3.h>
#endif

class MPU6050jb : public IMU {

public:
  MPU6050jb( int addr = 0x68 );
  void init( void );
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

  int16_t gyro_x, gyro_y, gyro_z;
  int16_t acc_x, acc_y, acc_z; // Changed long -> int16_t
  long acc_total_vector;
  int16_t temperature;
  long gyro_x_cal, gyro_y_cal, gyro_z_cal;
  float angle_pitch, angle_roll;
  int16_t angle_pitch_buffer, angle_roll_buffer;
  boolean set_gyro_angles;
  float angle_roll_acc, angle_pitch_acc;
  float angle_pitch_output, angle_roll_output;
  
};
