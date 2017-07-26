#include "Atm_mc_mixer.hpp"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

Atm_mc_mixer& Atm_mc_mixer::begin( int personality /* = CFG_QUADX */ ) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*          ON_ENTER    ON_LOOP  ON_EXIT  EVT_START  EVT_STOP  ELSE */
    /*  IDLE */ ENT_IDLE, ATM_SLEEP,      -1,       RUN,       -1,   -1,
    /*   RUN */  ENT_RUN, ATM_SLEEP,      -1,        -1,     IDLE,   -1,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  config( CFG_QUADX );
  return *this;          
}

Atm_mc_mixer& Atm_mc_mixer::config( int personality /* = CFG_QUADX */ ) {
  switch( personality ) {
    case CFG_QUADX:
      // Configuration for a standard X-quadcopter 
      // Motor order: FR(ccw), 1=RR(cw), 2=RL(ccw), 3=FL(cw)
      // Input order: yaw, pitch, roll, throttle
      mix( 0, -100, -100, -100, +100 ); 
      mix( 1, +100, +100, -100, +100 ); 
      mix( 2, -100, +100, +100, +100 ); 
      mix( 3, +100, -100, +100, +100 );
      input( -250, 250 );
      input( 3, 0, 1000 );
      output( -1, -1 );
      master( 3 );
      break;
  }
  return *this;
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int Atm_mc_mixer::event( int id ) {
  switch ( id ) {
  }
  return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 *
 * Available connectors:
 *   push( connectors, ON_CHANGE, 0, <v>, <up> );
 */

void Atm_mc_mixer::action( int id ) {
  switch ( id ) {
    case ENT_IDLE:
      for ( int output_ch = 0; output_ch < NO_OF_OUTPUT_CHANNELS; output_ch++ ) { // Set all speeds to 0
        
      }
      break;
    case ENT_RUN:
      update_outputs();
      break;  
  }
}

// Sets the input channel mix for an output channel (motor) and enables the channel

Atm_mc_mixer& Atm_mc_mixer::mix( int output_ch, int8_t input_ch0, int8_t input_ch1, int8_t input_ch2, int8_t input_ch3 ) {
  output_channel[output_ch].mix[0] = input_ch0;
  output_channel[output_ch].mix[1] = input_ch1;
  output_channel[output_ch].mix[2] = input_ch2;
  output_channel[output_ch].mix[3] = input_ch3;
  return *this;
}

// Sets an output channel as master ( -1 = disabled )

Atm_mc_mixer& Atm_mc_mixer::master( int input_ch ) {
  this->master_input = input_ch;
  return *this;
}

// Calculates the output channel value according to the inputs and the configured mix

int Atm_mc_mixer::calculate_output( int output_ch ) { 
  int v = 0;
  for ( int input_ch = 0; input_ch < NO_OF_INPUT_CHANNELS; input_ch++ )
    v += input_channel[input_ch].value * ( output_channel[output_ch].mix[input_ch] / 100.0 );
  if ( master_input > -1 && input_channel[master_input].value == 0 ) v = 0;
  return constrain( v, 0, 1000 ); 
}

// Updates all output channels and optionally call the update_motors() method

void Atm_mc_mixer::update_outputs() {
  int8_t change_cnt = 0;
  for ( int output_ch = 0; output_ch < NO_OF_OUTPUT_CHANNELS; output_ch++ ) {
    if ( output_channel[output_ch].id_channel > -1 ) {
      int new_value = calculate_output( output_ch );
      new_value = map( new_value , 0, 1000, output_min, output_max );        
      if ( new_value != output_channel[output_ch].last_output ) {
        output_channel[output_ch].last_output = new_value;
        change_cnt++;
      }
    }
  }
  if ( change_cnt ) update_motors();
}

Atm_mc_mixer& Atm_mc_mixer::motors( int8_t pin0, int8_t pin1, int8_t pin2, int8_t pin3, int8_t pin4, int8_t pin5, int8_t pin6, int8_t pin7 ) {
  if ( pin0 > -1 ) output_channel[0].id_channel = pulse400.attach( pin0 );
  if ( pin1 > -1 ) output_channel[1].id_channel = pulse400.attach( pin1 );
  if ( pin2 > -1 ) output_channel[2].id_channel = pulse400.attach( pin2 );
  if ( pin3 > -1 ) output_channel[3].id_channel = pulse400.attach( pin3 );
  if ( pin4 > -1 ) output_channel[4].id_channel = pulse400.attach( pin4 );
  if ( pin5 > -1 ) output_channel[5].id_channel = pulse400.attach( pin5 );
  if ( pin6 > -1 ) output_channel[6].id_channel = pulse400.attach( pin6 );
  if ( pin7 > -1 ) output_channel[7].id_channel = pulse400.attach( pin7 );
  for ( int ch = 0; ch < NO_OF_OUTPUT_CHANNELS; ch++ ) 
    if ( output_channel[ch].id_channel > -1 && ch > last_motor ) last_motor = ch;
  return *this;
}

Atm_mc_mixer& Atm_mc_mixer::frequency( uint8_t freqmask, int16_t period /* = 2500 */ ) {
  pulse400.frequency( freqmask, period );
  return *this;
}

void Atm_mc_mixer::update_motors() {
  for ( int ch = 0; ch < NO_OF_OUTPUT_CHANNELS; ch++ ) 
    if ( output_channel[ch].id_channel > -1 ) {
      pulse400.set_pulse( 
        output_channel[ch].id_channel, 
        map( constrain( output_channel[ch].last_output, 0, 1000 ), 0, 1000, output_min, output_max ), 
            ch != last_motor );
  }
}

// Configures the output range for all output channels

Atm_mc_mixer& Atm_mc_mixer::output( int min, int max ) {
  this->output_min = min;
  this->output_max = max;
  return *this;
}

// Configures the input range for all input channels

Atm_mc_mixer& Atm_mc_mixer::input( int min, int max ) {
  for ( int input_ch = 0; input_ch < NO_OF_INPUT_CHANNELS; input_ch++ ) {
    input_channel[input_ch].min = min;
    input_channel[input_ch].max = max;
  }
  return *this;
}

// Configures the input range for a single input channel

Atm_mc_mixer& Atm_mc_mixer::input( int input_ch, int min, int max ) {
  input_channel[input_ch].min = min;
  input_channel[input_ch].max = max;
  return *this;
}

// Sets a single input channel value

Atm_mc_mixer& Atm_mc_mixer::set( int input_ch, int value ) {
  input_channel[input_ch].raw = value;
  value = constrain( value, 0, 1000 ); // document this!
  input_channel[input_ch].value = map( value, 0, 1000, input_channel[input_ch].min, input_channel[input_ch].max ); 
  if ( state() ) update_outputs();
  return *this;
}

int Atm_mc_mixer::get( int input_ch, bool raw /* = false */ ) {
  return raw ? input_channel[input_ch].raw : input_channel[input_ch].value;
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

Atm_mc_mixer& Atm_mc_mixer::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int Atm_mc_mixer::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */

Atm_mc_mixer& Atm_mc_mixer::start() {
  trigger( EVT_START );
  return *this;
}

Atm_mc_mixer& Atm_mc_mixer::stop() {
  trigger( EVT_STOP );
  return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

Atm_mc_mixer& Atm_mc_mixer::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "FC_MIXER\0EVT_START\0EVT_STOP\0ELSE\0IDLE\0RUN" );
  return *this;
}



