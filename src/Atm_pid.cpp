#include "Atm_pid.hpp"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

Atm_pid& Atm_pid::begin( int sample_rate_ms, float Kp, float Ki, float Kd, float windup ) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*               ON_ENTER    ON_LOOP  ON_EXIT  EVT_TIMER  EVT_START  EVT_STOP  EVT_CHANGED  ELSE */
    /*    IDLE */          -1, ATM_SLEEP,      -1,        -1,       RUN,       -1,          -1,   -1,
    /*     RUN */          -1,        -1,      -1,    SAMPLE,        -1,     IDLE,          -1,   -1,
    /*  SAMPLE */  ENT_SAMPLE,        -1,      -1,        -1,        -1,     IDLE,     CHANGED,  RUN,
    /* CHANGED */ ENT_CHANGED,        -1,      -1,        -1,        -1,     IDLE,          -1,  RUN,
  };
  // clang-format on
  timer.set( sample_rate_ms );
  Machine::begin( state_table, ELSE );
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  windup_guard = windup;
  reset();
  return *this;          
}

Atm_pid& Atm_pid::reset() {
  setPoint = 0;
  integral = 0;
  derivative = 0;
  last_error = 0;
  controlVariable = 0.0; 
  return *this;
}


/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int Atm_pid::event( int id ) {
  switch ( id ) {
    case EVT_TIMER:
      return timer.expired( this );
    case EVT_CHANGED:
      return controlVariable != last_cv;
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

void Atm_pid::action( int id ) {
  float error;
  switch ( id ) {
    case ENT_SAMPLE:
      connectors[ON_SAMPLE].push( 0, 0 ); 
      error = setPoint - processVariable;
      integral += error * ( timer.value / 1000.0 );    
      integral = constrain( integral, -windup_guard, +windup_guard ); 
      derivative = ( error - last_error ) / float( timer.value );    
      controlVariable = ( Kp * error ) + ( Ki * integral ) + ( Kd * derivative );   
      last_error = error;        
      return;
    case ENT_CHANGED:
      connectors[ON_CHANGE].push( controlVariable * 10000, 0 );
      last_cv = controlVariable;
      return;
  }
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

Atm_pid& Atm_pid::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

Atm_pid& Atm_pid::sp( float setPoint ) {
  if ( Machine::state() ) {
    this->setPoint = setPoint;
  } else {
    connectors[ON_CHANGE].push( setPoint * 10000, 0 ); // Open loop mode (CV = SP)
  }
  return *this;
}

Atm_pid& Atm_pid::pv( float processVariable ) {
  this->processVariable = processVariable;
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int Atm_pid::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */

Atm_pid& Atm_pid::start() {
  trigger( EVT_START );
  return *this;
}

Atm_pid& Atm_pid::stop() {
  trigger( EVT_STOP );
  return *this;
}


/*
 * onChange() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

Atm_pid& Atm_pid::onChange( Machine& machine, int event ) {
  onPush( connectors, ON_CHANGE, 0, 1, 1, machine, event );
  return *this;
}

Atm_pid& Atm_pid::onChange( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_CHANGE, 0, 1, 1, callback, idx );
  return *this;
}

/*
 * onSample() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

Atm_pid& Atm_pid::onSample( Machine& machine, int event ) {
  onPush( connectors, ON_SAMPLE, 0, 1, 1, machine, event );
  return *this;
}

Atm_pid& Atm_pid::onSample( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_SAMPLE, 0, 1, 1, callback, idx );
  return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

Atm_pid& Atm_pid::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "PID\0EVT_TIMER\0EVT_START\0EVT_STOP\0EVT_CHANGED\0ELSE\0IDLE\0RUN\0SAMPLE\0CHANGED" );
  return *this;
}



