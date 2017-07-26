#pragma once

#include <Automaton.h>
#include <Pulse400.h>

#define NO_OF_INPUT_CHANNELS 4
#define NO_OF_OUTPUT_CHANNELS 8

// TODO: change output to 3/4 arguments:
// output( min, max, min_running ): min_running = minimum als throttle > 0
// zodat motoren altjd blijven draaien als de throttle > 0
// zie: http://autoquad.org/wiki/wiki/configuring-autoquad-flightcontroller/esc-calibration/

// Save RAM by setting only global output min, max

typedef struct {
    int min, max;
    int value, raw; // We could drop the raw and save 4 * 2 = 8 bytes!
    bool master;
} input_channel_struct;

typedef struct {
    int value;    
    int8_t mix[NO_OF_INPUT_CHANNELS];
} output_channel_struct;

class Atm_mc_mixer: public Machine {

 public:
  enum { IDLE, RUN }; // STATES
  enum { EVT_START, EVT_STOP, EVT_UPDATE, ELSE }; // EVENTS
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
  int read( int output_ch );
  Atm_mc_mixer& motors( int8_t pin0 = -1, int8_t pin1 = -1, int8_t pin2 = -1, int8_t pin3 = -1, int8_t pin4 = -1, int8_t pin5 = -1, int8_t pin6 = -1, int8_t pin7 = -1 );
  Atm_mc_mixer& mix( int output_ch, int8_t input_ch0, int8_t input_ch1, int8_t input_ch2, int8_t input_ch3 );
  Atm_mc_mixer& mix( int output_ch );
  Atm_mc_mixer& input( int input_ch, int min, int max );
  Atm_mc_mixer& input( int min, int max );
  Atm_mc_mixer& output( int min, int max );
  Atm_mc_mixer& master( int input_ch );
  Atm_mc_mixer& frequency( uint8_t freqmask, int16_t period = 2500 );

 private:
  enum { ENT_IDLE, ENT_RUN }; // ACTIONS
  enum { CONN_MAX }; // CONNECTORS
  int event( int id ); 
  void action( int id ); 
  int calculate_output( int output_ch );
  void update_outputs( void );
  void update_motors( void );
  input_channel_struct input_channel[NO_OF_INPUT_CHANNELS];
  output_channel_struct output_channel[NO_OF_OUTPUT_CHANNELS];
  byte master_input;
  int8_t last_motor = 0;
  int16_t output_min = 1000, output_max = 2000;
};


