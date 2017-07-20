#pragma once

#include <Automaton.h>

// Is it useful to be able to set master mode for each channel???

#include "Wire.h"

typedef struct {
    int16_t value, last_value;
    int16_t last_output;    
    int16_t min_out, max_out; // , offset;
    int16_t rate_pos;
    byte logical, reverse, master;
    int gyro;
    int acc
    int32_t gyro_cal;
    float angle, angle_output;
} axis_struct_jb;

class Atm_mpu6050jb: public Machine {

 public:
  enum { YAW, PITCH, ROLL };
  enum { REVERSE = B10000000 };
  enum { IDLE, INIT, CAL, SAMPLE, CHANGED }; // STATES
  enum { EVT_SAMPLE, EVT_CHANGE, EVT_TIMER, EVT_COUNTER, EVT_START, EVT_STOP, EVT_INITDONE, ELSE }; // EVENTS
  Atm_mpu6050jb( void ) : Machine() {};
  Atm_mpu6050jb& begin( int sample_rate );
  Atm_mpu6050jb& trace( Stream & stream );
  Atm_mpu6050jb& trigger( int event );
  int state( void );
  Atm_mpu6050jb& start( void );
  Atm_mpu6050jb& stop( void );
  
  Atm_mpu6050jb& onChange( Machine& machine, int event = 0 );
  Atm_mpu6050jb& onChange( atm_cb_push_t callback, int idx = 0 );
  Atm_mpu6050jb& onChange( int sub, Machine& machine, int event = 0 );
  Atm_mpu6050jb& onChange( int sub, atm_cb_push_t callback, int idx = 0 );
  Atm_mpu6050jb& onStabilize( Machine& machine, int event = 0 );
  Atm_mpu6050jb& onStabilize( atm_cb_push_t callback, int idx = 0 );

  int read( int ypr );
  int rate( int ypr );
  int rate( void );
  Atm_mpu6050jb& range( int ypr, int toLow, int toHigh );
  Atm_mpu6050jb& range( int toLow, int toHigh );
  Atm_mpu6050jb& angle( int ypr, int max_angle );
  Atm_mpu6050jb& angle( int max_angle );
  Atm_mpu6050jb& mapping( int axis0, int axis1, int axis2 );
  Atm_mpu6050jb& calibrate( int ypr, int v );
  Atm_mpu6050jb& calibrate( int ypr );
  Atm_mpu6050jb& stabilize( uint16_t win_size = 5, uint16_t win_millis = 5000 );
  Atm_mpu6050jb& master( int ypr, bool master = true );
  Atm_mpu6050jb& master( bool master = true );

  private:
  enum { ENT_INIT, ENT_SAMPLE, ENT_CHANGED, ENT_CAL }; // ACTIONS
  enum { ON_CHANGE, ON_STABILIZE = 3, CONN_MAX }; // CONNECTORS
  atm_connector connectors[CONN_MAX];
  int event( int id ); 
  void action( int id );
  void setup_mpu_6050_registers();
  void read_mpu_6050_data();                                             
  
  atm_timer_millis timer;
  atm_counter init_counter;
  
  uint16_t packetSize, fifoCount;
  axis_struct_jb axis[3];
  byte physical[3];  
  
  bool enable_stabilize;
  int16_t rate_win, rate_pos, rate_fin_counter, rate_cur_counter, rate_millis; 
  byte rate_cur_second;
  int temperature;
  bool set_gyro_angles;
  
  int cnt;
};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_mpu6050jb">
    <states>
      <IDLE index="0" sleep="1">
        <EVT_START>INIT</EVT_START>
      </IDLE>
      <INIT index="1" on_enter="ENT_INIT">
        <EVT_STOP>IDLE</EVT_STOP>
        <EVT_INITDONE>RUN</EVT_INITDONE>
      </INIT>
      <RUN index="2" on_enter="ENT_RUN">
        <EVT_CHANGE>CHANGE</EVT_CHANGE>
        <EVT_TIMER>RUN</EVT_TIMER>
        <EVT_STOP>IDLE</EVT_STOP>
      </RUN>
      <CHANGE index="3">
        <EVT_TIMER>RUN</EVT_TIMER>
        <EVT_STOP>IDLE</EVT_STOP>
      </CHANGE>
    </states>
    <events>
      <EVT_CHANGE index="0" access="PRIVATE"/>
      <EVT_TIMER index="1" access="PRIVATE"/>
      <EVT_START index="2" access="PUBLIC"/>
      <EVT_STOP index="3" access="PUBLIC"/>
      <EVT_INITDONE index="4" access="PRIVATE"/>
    </events>
    <connectors>
      <CHANGE autostore="0" broadcast="0" dir="PUSH" slots="1"/>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/

