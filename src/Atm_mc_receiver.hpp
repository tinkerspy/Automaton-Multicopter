#pragma once

#include <Automaton.h>

typedef struct {
    uint8_t pin;
    uint16_t value;
    uint16_t last_value;    
    uint16_t last_output;    
    uint32_t last_high;
    uint16_t min, max, min_in, max_in, min_out, max_out;
    uint16_t sticky;
    int logical;
} rc_struct;

#define CHANNELS 6

// TODO overnemen uit Atm_mpu6050: connectorsetup en p2l vervangen door structveld

class Atm_mc_receiver: public Machine {

 public:
  enum { IDLE, WAIT, PAUSE, RUN, CHANGED }; // STATES
  enum { EVT_WAIT, EVT_TIMER,  EVT_START, EVT_STOP, EVT_TOGGLE, EVT_CHANGED, ELSE }; // EVENTS
  Atm_mc_receiver( void ) : Machine() {};
  Atm_mc_receiver& begin(  int p0, int p1 = -1, int p2 = -1, int p3 = -1, int p4 = -1, int p5 = -1  );
  Atm_mc_receiver& trace( Stream & stream );
  Atm_mc_receiver& trigger( int event );
  int state( void );
  void handleInterruptPWM( int pch );
  void handleInterruptPPM();
  int read( int lch, bool raw = 0 );
  Atm_mc_receiver& ppm( void );
  Atm_mc_receiver& pwm( void );
  Atm_mc_receiver& start( void );
  Atm_mc_receiver& stop( void );
  Atm_mc_receiver& toggle( void );
  Atm_mc_receiver& reset( int lch = -1 );
  Atm_mc_receiver& sticky( int lch, int value );
  Atm_mc_receiver& sticky( int value );
  int minimum( int lch );
  int maximum( int lch );  
  Atm_mc_receiver& calibrate( int idx, int min, int max );

  Atm_mc_receiver& onChange( Machine& machine, int event = 0 );
  Atm_mc_receiver& onChange( atm_cb_push_t callback, int idx = 0 );
  Atm_mc_receiver& onChange( int sub, Machine& machine, int event = 0 );
  Atm_mc_receiver& onChange( int sub, atm_cb_push_t callback, int idx = 0 );

  
  Atm_mc_receiver& mapping( int pch0 = -1, int pch1 = -1, int pch2 = -1, int pch3 = -1, int pch4 = -1, int pch5 = -1 );
  int volatile intCount = 0;

 private:
  enum { ENT_CHANGED }; // ACTIONS
  enum { ON_CHANGE, CONN_MAX = CHANNELS }; // CONNECTORS
  atm_connector connectors[CONN_MAX];

  int event( int id ); 
  void action( int id );
  int translate( int idx ); 
  rc_struct volatile channel[CHANNELS];  
  static Atm_mc_receiver * instance;
  uint8_t volatile ppm_pulse_counter;
  uint32_t volatile ppm_last_pulse;
  uint8_t max_used_channel;
  int physical[CHANNELS]; // make it a uint8_t???
  atm_timer_millis timer; // Wait for RC to stabilize
  
};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_mc_receiver">
    <states>
      <IDLE index="0" sleep="1">
      </IDLE>
    </states>
    <events>
    </events>
    <connectors>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/

