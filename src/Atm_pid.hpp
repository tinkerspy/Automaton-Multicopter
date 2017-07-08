#pragma once

#include <Automaton.h>

class Atm_pid: public Machine {

 public:
  enum { IDLE, RUN, SAMPLE, CHANGED }; // STATES
  enum { EVT_TIMER, EVT_START, EVT_STOP, EVT_CHANGED, ELSE }; // EVENTS
  Atm_pid( void ) : Machine() {};
  Atm_pid& begin( int sample_rate, float Kp = 0.2, float Ki = 0.0, float Kd = 0.0, float windup = 20.0 );
  Atm_pid& trace( Stream & stream );
  Atm_pid& trigger( int event );
  int state( void );
  Atm_pid& onChange( Machine& machine, int event = 0 );
  Atm_pid& onChange( atm_cb_push_t callback, int idx = 0 );
  Atm_pid& onSample( Machine& machine, int event = 0 );
  Atm_pid& onSample( atm_cb_push_t callback, int idx = 0 );
  Atm_pid& start( void );
  Atm_pid& stop( void );
  Atm_pid& sp( float setPoint );
  Atm_pid& pv( float processValue );
  Atm_pid& Kp( float KpValue );
  Atm_pid& Ki( float KiValue );
  Atm_pid& Kd( float KdValue );
  Atm_pid& windup( float v );
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
  enum { ON_CHANGE, ON_SAMPLE, CONN_MAX }; // CONNECTORS
  atm_connector connectors[CONN_MAX];
  int event( int id ); 
  void action( int id ); 
  float calculate( float setPoint, float processVariable );
  atm_timer_millis timer;
  float KpValue, KiValue, KdValue;
  float setPoint, processVariable, controlVariable, last_cv;
  float integral, derivative, last_error; 
  float windup_guard;


};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_pid">
    <states>
      <IDLE index="0" sleep="1">
        <EVT_START>RUN</EVT_START>
      </IDLE>
      <RUN index="1">
        <EVT_TIMER>SAMPLE</EVT_TIMER>
        <EVT_STOP>IDLE</EVT_STOP>
      </RUN>
      <SAMPLE index="2" on_enter="ENT_SAMPLE">
        <EVT_STOP>IDLE</EVT_STOP>
        <EVT_CHANGED>CHANGED</EVT_CHANGED>
        <ELSE>RUN</ELSE>
      </SAMPLE>
      <CHANGED index="3" on_enter="ENT_CHANGED">
        <EVT_STOP>IDLE</EVT_STOP>
      </CHANGED>
    </states>
    <events>
      <EVT_TIMER index="0" access="PRIVATE"/>
      <EVT_START index="1" access="PUBLIC"/>
      <EVT_STOP index="2" access="PUBLIC"/>
      <EVT_CHANGED index="3" access="MIXED"/>
    </events>
    <connectors>
      <CHANGE autostore="0" broadcast="0" dir="PUSH" slots="1"/>
      <SAMPLE autostore="0" broadcast="0" dir="PUSH" slots="1"/>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/

