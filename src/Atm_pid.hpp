#pragma once

#include <Automaton.h>

/*

IDLE - Open loop mode, sp is immediately copied to cv, onChange() connector is called
RUN - Closed loop mode, timer or master based samples, onChange() called only for changes
HOLD - Closed loop mode with frozen sp

*/


class Atm_pid: public Machine {

 public:
  enum { IDLE, RUN, SAMPLE, CHANGED }; // STATES
  enum { EVT_TIMER, EVT_START, EVT_STOP, EVT_CHANGED, ELSE }; // EVENTS
  Atm_pid( void ) : Machine() {};
  Atm_pid& begin( int sample_rate = -1, float Kp = 0.2, float Ki = 0.0, float Kd = 0.0, float windup = 20.0 );
  Atm_pid& trace( Stream & stream );
  Atm_pid& trigger( int event );
  int state( void );
  Atm_pid& onUpdate( Machine& machine, int event = 0 );
  Atm_pid& onUpdate( atm_cb_push_t callback, int idx = 0 );
  Atm_pid& onSample( Machine& machine, int event = 0 );
  Atm_pid& onSample( atm_cb_push_t callback, int idx = 0 );
  Atm_pid& start( void );
  Atm_pid& stop( void );
  Atm_pid& sp( float setPoint );
  Atm_pid& pv( float processValue );
  Atm_pid& pid( float KpValue, float KiValue = 0, float KdValue = 0 );
  Atm_pid& Kp( float KpValue );
  Atm_pid& Ki( float KiValue );
  Atm_pid& Kd( float KdValue );
  Atm_pid& windup( float v );
  Atm_pid& offset( float v );
  float sp( void );
  float pv( void );
  float cv( void );
  float Kp( void );
  float Ki( void );
  float Kd( void );
  float windup( void );
  Atm_pid& reset();
  
 private:
  enum { ENT_SAMPLE, ENT_CHANGED }; // ACTIONS
  enum { ON_UPDATE, ON_SAMPLE, CONN_MAX }; // CONNECTORS
  atm_connector connectors[CONN_MAX];
  int event( int id ); 
  void action( int id ); 
  float calculate( float setPoint, float processVariable );
  atm_timer_millis timer;
  float KpValue, KiValue, KdValue;
  float setPoint, processVariable, controlVariable, last_cv;
  float integral, derivative, last_error; 
  float windup_guard, output_offset;
  uint32_t last_calculation;

};

