#include "Atm_filter.hpp"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

  Atm_filter& Atm_filter::begin( uint32_t sample_rate_ms, uint16_t* v, uint16_t size ) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*               ON_ENTER  ON_LOOP  ON_EXIT  EVT_CHANGED  EVT_TIMER  ELSE */
    /*    IDLE */          -1,      -1,      -1,          -1,    SAMPLE,   -1,
    /*  SAMPLE */  ENT_SAMPLE,      -1,      -1,     CHANGED,        -1,   -1,
    /* CHANGED */ ENT_CHANGED,      -1,      -1,          -1,        -1, IDLE,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  avg_buf = v;
  avg_buf_size = size / sizeof( int16_t );
  avg_buf_head = 0;
  avg_buf_total = 0;
  first_run = true;
  /*
  for ( uint16_t i = 0; i < avg_buf_size; i++ ) {
    avg_buf[i] = read_sample();
    avg_buf_total += avg_buf[i];
  }
  */
  timer.set( sample_rate_ms );
  return *this;          
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int Atm_filter::event( int id ) {
  switch ( id ) {
    case EVT_CHANGED:
      return 0;
    case EVT_TIMER:
      return timer.expired( this );
  }
  return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 *
 * Available connectors:
 *   push( connectors, ON_CHANGE, 0, <v>, <up> );
 *   push( connectors, ON_SAMPLE, 0, <v>, <up> );
 */

void Atm_filter::action( int id ) {
  switch ( id ) {
    case ENT_SAMPLE:
      return;
    case ENT_CHANGED:
      return;
  }
}


Atm_filter& Atm_filter::sample( int v ) {
  last_sample = v;
  return *this;
}




/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

Atm_filter& Atm_filter::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int Atm_filter::state( void ) {
  return Machine::state();
}



/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */

/*
 * onChange() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

Atm_filter& Atm_filter::onChange( Machine& machine, int event ) {
  onPush( connectors, ON_CHANGE, 0, 1, 1, machine, event );
  return *this;
}

Atm_filter& Atm_filter::onChange( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_CHANGE, 0, 1, 1, callback, idx );
  return *this;
}

/*
 * onSample() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

Atm_filter& Atm_filter::onSample( Machine& machine, int event ) {
  onPush( connectors, ON_SAMPLE, 0, 1, 1, machine, event );
  return *this;
}

Atm_filter& Atm_filter::onSample( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_SAMPLE, 0, 1, 1, callback, idx );
  return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

Atm_filter& Atm_filter::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "FILTER\0EVT_CHANGED\0EVT_TIMER\0ELSE\0IDLE\0SAMPLE\0CHANGED" );
  return *this;
}



