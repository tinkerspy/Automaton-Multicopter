#pragma once

#include <Automaton.h>

#define NO_OF_INPUT_CHANNELS 4
#define NO_OF_OUTPUT_CHANNELS 8

// TODO: change ouput to 4 arguments:
// output( min, max, min_running ): min_running = minimum als throttle > 0
// zodat motoren altjd blijven draaien als de throttle > 0
// zie: http://autoquad.org/wiki/wiki/configuring-autoquad-flightcontroller/esc-calibration/

typedef struct {
    int min, max;
    int value, raw;
    bool master;
} input_channel_struct;

typedef struct {
    int last_output;    
    int min, max;
    int8_t mix[NO_OF_INPUT_CHANNELS];
    bool enabled;
} output_channel_struct;

class Atm_mc_mixer: public Machine {

 public:
  enum { IDLE, RUN }; // STATES
  enum { EVT_START, EVT_STOP, ELSE }; // EVENTS
  enum { CFG_MANUAL, CFG_QUADX };
  Atm_mc_mixer( void ) : Machine() {};
  Atm_mc_mixer& begin(  int personality = CFG_QUADX );
  Atm_mc_mixer& config( int personality = CFG_QUADX );
  Atm_mc_mixer& trace( Stream & stream );
  Atm_mc_mixer& trigger( int event );
  int state( void );
  Atm_mc_mixer& onChange( Machine& machine, int event = 0 );
  Atm_mc_mixer& onChange( atm_cb_push_t callback, int idx = 0 );
  Atm_mc_mixer& start( void );
  Atm_mc_mixer& stop( void );
  Atm_mc_mixer& set( int input_ch, int value );
  int get( int input_ch, bool raw = false );
  Atm_mc_mixer& mix( int output_ch, int8_t input_ch0, int8_t input_ch1, int8_t input_ch2, int8_t input_ch3 );
  Atm_mc_mixer& mix( int output_ch );
  Atm_mc_mixer& input( int input_ch, int min, int max );
  Atm_mc_mixer& input( int min, int max );
  Atm_mc_mixer& output( int output_ch, int min, int max );
  Atm_mc_mixer& output( int min, int max );
  Atm_mc_mixer& master( int input_ch );

 private:
  enum { ENT_IDLE, ENT_RUN }; // ACTIONS
  enum { ON_CHANGE, CONN_MAX }; // CONNECTORS
  atm_connector connectors[CONN_MAX];
  int event( int id ); 
  void action( int id ); 
  int calculate_output( int output_ch );
  void update_outputs( void );
  input_channel_struct input_channel[NO_OF_INPUT_CHANNELS];
  output_channel_struct output_channel[NO_OF_OUTPUT_CHANNELS];
  byte master_input;
};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_mc_mixer">
    <states>
      <IDLE index="0" sleep="1">
        <EVT_START>RUN</EVT_START>
      </IDLE>
      <RUN index="1" sleep="1">
        <EVT_STOP>IDLE</EVT_STOP>
      </RUN>
    </states>
    <events>
      <EVT_START index="0" access="PUBLIC"/>
      <EVT_STOP index="1" access="PUBLIC"/>
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

