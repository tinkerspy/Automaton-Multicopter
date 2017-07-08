#include "Atm_mc_mixer.hpp"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

Atm_mc_mixer& Atm_mc_mixer::begin( int personality /* = CFG_MIXER_QUADX */ ) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*          ON_ENTER    ON_LOOP  ON_EXIT  EVT_START  EVT_STOP  ELSE */
    /*  IDLE */ ENT_IDLE, ATM_SLEEP,      -1,       RUN,       -1,   -1,
    /*   RUN */  ENT_RUN, ATM_SLEEP,      -1,        -1,     IDLE,   -1,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  config( CFG_MIXER_QUADX );
  return *this;          
}

Atm_mc_mixer& Atm_mc_mixer::config( int personality /* = CFG_MIXER_QUADX */ ) {
  switch( personality ) {
    case CFG_MIXER_QUADX:
      // Configuration for a standard X-quadcopter 
      // Motor order: FR(ccw), 1=RR(cw), 2=RL(ccw), 3=FL(cw)
      // Input order: yaw, pitch, roll, throttle
      mix( 0, -1, -1, -1, +1 ); 
      mix( 1, +1, +1, -1, +1 ); 
      mix( 2, -1, +1, +1, +1 ); 
      mix( 3, +1, -1, +1, +1 );
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
        if ( output_channel[output_ch].enabled ) 
          connectors[ON_CHANGE].push( 0, output_ch );
      }
      break;
    case ENT_RUN:
      update_outputs();
      break;  
  }
}

// Sets the input channel mix for an output channel (motor) and enables the channel

Atm_mc_mixer& Atm_mc_mixer::mix( int output_ch, int input_ch0, int input_ch1, int input_ch2, int input_ch3 ) {
  output_channel[output_ch].mix[0] = input_ch0;
  output_channel[output_ch].mix[1] = input_ch1;
  output_channel[output_ch].mix[2] = input_ch2;
  output_channel[output_ch].mix[3] = input_ch3;
  output_channel[output_ch].enabled = true;
  return *this;
}

// Disables the output channel (motor)

Atm_mc_mixer& Atm_mc_mixer::mix( int output_ch ) {
  output_channel[output_ch].enabled = false;
  return *this;
}

// Sets an output channel as master ( -1 = disabled )

Atm_mc_mixer& Atm_mc_mixer::master( int input_ch ) {
  this->master_input = input_ch;
  return *this;
}

// Calculates the output channel value according to the inputs and the configured mix

int Atm_mc_mixer::calculate_output( int output_ch ) { // Add master role
  int v = 0;
  for ( int input_ch = 0; input_ch < NO_OF_INPUT_CHANNELS; input_ch++ )
    v += input_channel[input_ch].value * output_channel[output_ch].mix[input_ch];
  if ( master_input > -1 && input_channel[master_input].value == 0 ) v = 0;
  return constrain( v, 0, 1000 ); 
  /* TEST THE ABOVE LOGIC!!!
  return constrain( 
    ( input_channel[0].value * output_channel[output_ch].mix[0] ) +
    ( input_channel[1].value * output_channel[output_ch].mix[1] ) +
    ( input_channel[2].value * output_channel[output_ch].mix[2] ) +
    ( input_channel[3].value * output_channel[output_ch].mix[3] ), 0, 1000 );
  */
}

// Updates all output channels and optionally call the onChange() method

void Atm_mc_mixer::update_outputs() {
    for ( int output_ch = 0; output_ch < NO_OF_OUTPUT_CHANNELS; output_ch++ ) {
      if ( output_channel[output_ch].enabled ) {
        int new_value = calculate_output( output_ch );
        if ( new_value != output_channel[output_ch].last_output ) {
          output_channel[output_ch].last_output = new_value;
          if ( output_channel[output_ch].min != -1 && output_channel[output_ch].max != -1 ) 
            new_value = map( new_value , 0, 1000, output_channel[output_ch].min, output_channel[output_ch].max );
          connectors[ON_CHANGE].push( new_value, output_ch ); 
        }
      }
    }
}

// Configures the output range for all output channels

Atm_mc_mixer& Atm_mc_mixer::output( int min, int max ) {
  for ( int output_ch = 0; output_ch < NO_OF_OUTPUT_CHANNELS; output_ch++ ) {
    output_channel[output_ch].min = min;
    output_channel[output_ch].max = max;
  }
  return *this;
}

// Configures the output range for a single output channel

Atm_mc_mixer& Atm_mc_mixer::output( int output_ch, int min, int max ) {
  output_channel[output_ch].min = min;
  output_channel[output_ch].max = max;
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
  input_channel[input_ch].value = map( value, 0, 1000, input_channel[input_ch].min, input_channel[input_ch].max ); 
  if ( state() ) update_outputs();
  return *this;
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

/*
 * onChange() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

Atm_mc_mixer& Atm_mc_mixer::onChange( Machine& machine, int event ) {
  onPush( connectors, ON_CHANGE, 0, 1, 1, machine, event );
  return *this;
}

Atm_mc_mixer& Atm_mc_mixer::onChange( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_CHANGE, 0, 1, 1, callback, idx );
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



