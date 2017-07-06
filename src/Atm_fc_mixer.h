#pragma once

#include <Automaton.h>

#define NO_OF_INPUT_CHANNELS 4
#define NO_OF_OUTPUT_CHANNELS 10

// TODO: Configure the master!!!!

typedef struct {
    int min, max;
    int value, raw;
    bool master;
} input_channel_struct;

typedef struct {
    int last_output;    
    int min, max;
    float mix[NO_OF_INPUT_CHANNELS];
    int logical; // not used yet
    bool enabled;
} output_channel_struct;

class Atm_fc_mixer: public Machine {

 public:
  enum { IDLE, RUN }; // STATES
  enum { EVT_START, EVT_STOP, ELSE }; // EVENTS
  enum { CFG_MIXER_QUADX };
  Atm_fc_mixer( void ) : Machine() {};
  Atm_fc_mixer& begin(  int personality = CFG_MIXER_QUADX );
  Atm_fc_mixer& config( int personality = CFG_MIXER_QUADX );
  Atm_fc_mixer& trace( Stream & stream );
  Atm_fc_mixer& trigger( int event );
  int state( void );
  Atm_fc_mixer& onChange( Machine& machine, int event = 0 );
  Atm_fc_mixer& onChange( atm_cb_push_t callback, int idx = 0 );
  Atm_fc_mixer& start( void );
  Atm_fc_mixer& stop( void );
  Atm_fc_mixer& set( int input_ch, int value );
  Atm_fc_mixer& mix( int output_ch, int input_ch0, int input_ch1, int input_ch2, int input_ch3 );
  Atm_fc_mixer& mix( int output_ch );
  Atm_fc_mixer& input( int input_ch, int min, int max );
  Atm_fc_mixer& input( int min, int max );
  Atm_fc_mixer& output( int output_ch, int min, int max );
  Atm_fc_mixer& output( int min, int max );
  Atm_fc_mixer& master( int input_ch );

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

};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_fc_mixer">
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

