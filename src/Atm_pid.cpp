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
    /*  SAMPLE */  ENT_SAMPLE,        -1,      -1,        -1,        -1,     IDLE,          -1,  RUN,
    /* CHANGED */ ENT_CHANGED,        -1,      -1,        -1,        -1,     IDLE,          -1,  RUN,
  };
  // clang-format on
  timer.set( sample_rate_ms );
  Machine::begin( state_table, ELSE );
  this->KpValue = Kp;
  this->KiValue = Ki;
  this->KdValue = Kd;
  windup_guard = windup;
  output_offset = 0.0;
  setPoint = 0;
  reset();
  return *this;          
}

Atm_pid& Atm_pid::reset() {
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
  switch ( id ) {
    case ENT_SAMPLE:
      push( connectors, ON_SAMPLE, 0, 0, 0 ); 
      controlVariable = calculate( setPoint, processVariable );
      push( connectors, ON_UPDATE, 0, controlVariable + output_offset, 0 ); 
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

float Atm_pid::calculate( float setPoint, float processVariable ) {
  float error = setPoint - processVariable;
  if ( last_calculation ) {
    float dt = ( micros() - last_calculation ) / 1000000.0; // TESTME!
    integral += error * dt;    
    integral = constrain( integral, -windup_guard, +windup_guard ); 
    derivative = ( error - last_error ) / dt;
    last_calculation = micros();  
    last_error = error;      
  }  
  return ( KpValue * error ) + ( KiValue * integral ) + ( KdValue * derivative );   
}

Atm_pid& Atm_pid::sp( float setPoint ) { // TODO: While in HOLD ignore setPoint changes
  // No constrain() here, on purpose: pid is generic
  if ( Machine::state() ) {
    this->setPoint = setPoint;
  } else {
    this->setPoint = setPoint;
    this->controlVariable = setPoint;
    connectors[ON_UPDATE].push( setPoint, 0 ); // Open loop mode (CV = SP)
  }
  return *this;
}

Atm_pid& Atm_pid::pv( float processVariable ) {
  this->processVariable = processVariable;
  if ( timer.value == ATM_TIMER_OFF ) { 
  // Slave mode! (TODO: Moet ook nog state-afhankelijk zijn!)
  // IDLE? do nothing
//    Serial.print( "calc sp=");
//    Serial.print( setPoint );
//    Serial.print( " pv=");
//    Serial.print( this->processVariable );
    
    controlVariable = calculate( setPoint, processVariable );
//    Serial.print( " =");
//    Serial.println( this->controlVariable );
//    if ( controlVariable != last_cv ) {
      push( connectors, ON_UPDATE, 0, controlVariable + output_offset, 0 ); 
      last_cv = controlVariable;
//    }
  }
  return *this;
}

Atm_pid& Atm_pid::windup( float v ) {
  this->windup_guard = v;
  return *this;
}

Atm_pid& Atm_pid::offset( float v ) {
  this->output_offset = v;
  return *this;
}

float Atm_pid::windup( void ) {
  return this->windup_guard ;
}

float Atm_pid::sp( void ) {
  return this->setPoint;
}

float Atm_pid::pv( void ) {
  return this->processVariable;
}

Atm_pid& Atm_pid::Kp( float KpValue ) {
  this->KpValue = KpValue;
  return *this;  
}

Atm_pid& Atm_pid::Ki( float KiValue ) {
  this->KiValue = KiValue;
  return *this;  
}

Atm_pid& Atm_pid::Kd( float KdValue ) {
  this->KdValue = KdValue;
  return *this;  
}

Atm_pid& Atm_pid::pid( float KpValue, float KiValue, float KdValue ) {
  this->KpValue = KpValue;
  this->KiValue = KiValue;
  this->KdValue = KdValue;
  return *this;  
}

float Atm_pid::cv( void ) {
  return this->controlVariable + output_offset;
}

float Atm_pid::Kp( void ) {
  return this->KpValue;
}

float Atm_pid::Ki( void ) {
  return this->KiValue;
}

float Atm_pid::Kd( void ) {
  return this->KdValue;
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

Atm_pid& Atm_pid::onUpdate( Machine& machine, int event ) {
  onPush( connectors, ON_UPDATE, 0, 1, 1, machine, event );
  return *this;
}

Atm_pid& Atm_pid::onUpdate( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_UPDATE, 0, 1, 1, callback, idx );
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



