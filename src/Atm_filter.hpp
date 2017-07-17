#pragma once

#include <Automaton.h>

class Atm_filter: public Machine {

 public:
  enum { IDLE, SAMPLE, CHANGED }; // STATES
  enum { EVT_CHANGED, EVT_TIMER, ELSE }; // EVENTS
  Atm_filter( void ) : Machine() {};
  Atm_filter& begin( uint32_t sample_rate_ms, uint16_t* v, uint16_t size );
  Atm_filter& trace( Stream & stream );
  Atm_filter& trigger( int event );
  int state( void );
  Atm_filter& onChange( Machine& machine, int event = 0 );
  Atm_filter& onChange( atm_cb_push_t callback, int idx = 0 );
  Atm_filter& onSample( Machine& machine, int event = 0 );
  Atm_filter& onSample( atm_cb_push_t callback, int idx = 0 );
  Atm_filter& sample( int v );
  
 private:
  enum { ENT_SAMPLE, ENT_CHANGED }; // ACTIONS
  enum { ON_CHANGE, ON_SAMPLE, CONN_MAX }; // CONNECTORS
  atm_connector connectors[CONN_MAX];
  int event( int id ); 
  void action( int id ); 
  
  int last_sample, last_output;
  bool first_run;
  atm_timer_millis timer;
  
  uint16_t* avg_buf;
  uint16_t avg_buf_size;
  uint16_t avg_buf_head;
  uint32_t avg_buf_total;
};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_filter">
    <states>
      <IDLE index="0">
        <EVT_TIMER>SAMPLE</EVT_TIMER>
      </IDLE>
      <SAMPLE index="1" on_enter="ENT_SAMPLE">
        <EVT_CHANGED>CHANGED</EVT_CHANGED>
      </SAMPLE>
      <CHANGED index="2" on_enter="ENT_CHANGED">
        <ELSE>IDLE</ELSE>
      </CHANGED>
    </states>
    <events>
      <EVT_CHANGED index="0" access="PRIVATE"/>
      <EVT_TIMER index="1" access="PRIVATE"/>
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

