#pragma once

#include <Automaton.h>
#include <imu/IMU.hpp>

typedef struct {
    int16_t value, last_value;
    int16_t last_output;    
    int16_t min_out, max_out, offset;
    byte logical, reverse;
} ag_axis_struct;


class Atm_mc_accelgyro: public Machine {

 public:
  enum { IDLE, RUN, SAMPLE, READING }; // STATES
  enum { EVT_TIMER, EVT_READ, EVT_START, EVT_STOP, ELSE }; // EVENTS
  Atm_mc_accelgyro( void ) : Machine() {};
  Atm_mc_accelgyro& begin(  IMU & imu, uint32_t sample_rate_us = -1 );
  Atm_mc_accelgyro& trace( Stream & stream );
  Atm_mc_accelgyro& trigger( int event );
  int state( void );
  Atm_mc_accelgyro& onChange( Machine& machine, int event = 0 );
  Atm_mc_accelgyro& onChange( atm_cb_push_t callback, int idx = 0 );
  Atm_mc_accelgyro& onChange( int sub, Machine& machine, int event = 0 );
  Atm_mc_accelgyro& onChange( int sub, atm_cb_push_t callback, int idx = 0 );
  Atm_mc_accelgyro& onUpdate( Machine& machine, int event = 0 );
  Atm_mc_accelgyro& onUpdate( atm_cb_push_t callback, int idx = 0 );
  Atm_mc_accelgyro& start( void );
  Atm_mc_accelgyro& stop( void );

  int read( int ypr );
  Atm_mc_accelgyro& range( int ypr, int toLow, int toHigh );
  Atm_mc_accelgyro& range( int toLow, int toHigh );
  Atm_mc_accelgyro& angle( int ypr, int max_angle );
  Atm_mc_accelgyro& angle( int max_angle );
  Atm_mc_accelgyro& mapping( int axis0, int axis1, int axis2 );
  Atm_mc_accelgyro& calibrate( int ypr, int v );
  Atm_mc_accelgyro& calibrate( int ypr );
  
 private:
  enum { YAW, PITCH, ROLL };
  enum { REVERSE = B10000000 };
  enum { ENT_IDLE, ENT_RUN, ENT_SAMPLE, ENT_READING }; // ACTIONS
  enum { ON_CHANGE, ON_UPDATE = 3, CONN_MAX }; // CONNECTORS
  atm_connector connectors[CONN_MAX];
  int event( int id ); 
  void action( int id );

  ag_axis_struct axis[3];
  byte physical[3]; 

  uint32_t microtimer, microtimer_value;
  IMU *imu;

  
};


