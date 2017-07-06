#pragma once

#include <Automaton.h>


#include "helper_3dmath.h"
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#include "MPU6050.h"

typedef struct {
    float value, last_value;
    int last_output;    
    int min_out, max_out;
    int logical, reverse;
} axis_struct;

class Atm_mpu6050: public Machine {

 public:
  enum { YAW, PITCH, ROLL };
  enum { REVERSE = B10000000 };
  enum { IDLE, INIT, RUN, CHECK, SAMPLE, CHANGED }; // STATES
  enum { EVT_SAMPLE, EVT_CHANGE, EVT_TIMER, EVT_START, EVT_STOP, EVT_INITDONE, ELSE }; // EVENTS
  Atm_mpu6050( void ) : Machine() {};
  Atm_mpu6050& begin( int sample_rate );
  Atm_mpu6050& trace( Stream & stream );
  Atm_mpu6050& trigger( int event );
  int state( void );
  Atm_mpu6050& start( void );
  Atm_mpu6050& stop( void );
  
  Atm_mpu6050& onChange( Machine& machine, int event = 0 );
  Atm_mpu6050& onChange( atm_cb_push_t callback, int idx = 0 );
  Atm_mpu6050& onChange( int sub, Machine& machine, int event = 0 );
  Atm_mpu6050& onChange( int sub, atm_cb_push_t callback, int idx = 0 );

  int read( int ypr );
  int rate( int ypr );
  int rate( void );
  Atm_mpu6050& range( int ypr, int toLow, int toHigh );
  Atm_mpu6050& range( int toLow, int toHigh );
  Atm_mpu6050& angle( int ypr, int max_angle );
  Atm_mpu6050& angle( int max_angle );
  Atm_mpu6050& mapping( int axis0, int axis1, int axis2 );
  

 private:
  enum { ENT_INIT, ENT_SAMPLE, ENT_CHECK, ENT_RUN, ENT_CHANGED }; // ACTIONS
  enum { ON_CHANGE, CONN_MAX = 3 }; // CONNECTORS
  atm_connector connectors[CONN_MAX];
  int event( int id ); 
  void action( int id );
  
  MPU6050 mpu6050;
  atm_timer_millis timer;

  uint8_t devStatus, mpuIntStatus;
  uint16_t packetSize, fifoCount;
  uint8_t fifoBuffer[64];
  bool dmpReady = false;
  axis_struct axis[3];
  int physical[3];  
};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_mpu6050">
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

