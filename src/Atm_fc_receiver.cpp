#include "Atm_fc_receiver.hpp"

// WARNING: You can only run one instance of this class per sketch 

// TODO: Smoothen startup (suppress first RC signal)

Atm_fc_receiver& Atm_fc_receiver::begin( int p0, int p1, int p2, int p3, int p4, int p5 ) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*                ON_ENTER    ON_LOOP  ON_EXIT  EVT_WAIT EVT_TIMER  EVT_START EVT_STOP EVT_TOGGLE EVT_CHANGED ELSE */
    /*  IDLE    */          -1, ATM_SLEEP,      -1,       -1,       -1,     WAIT,      -1,       RUN,         -1,  -1,
    /*  WAIT    */          -1,        -1,      -1,    PAUSE,       -1,       -1,    IDLE,      IDLE,         -1,  -1,
    /*  PAUSE   */          -1,        -1,      -1,       -1,      RUN,       -1,    IDLE,      IDLE,         -1,  -1,
    /*  RUN     */          -1,        -1,      -1,       -1,       -1,       -1,    IDLE,      IDLE,    CHANGED,  -1,
    /*  CHANGED */ ENT_CHANGED,        -1,      -1,       -1,       -1,       -1,    IDLE,      IDLE,         -1, RUN,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  channel[0].pin = p0;
  channel[1].pin = p1;
  channel[2].pin = p2;
  channel[3].pin = p3;
  channel[4].pin = p4;
  channel[5].pin = p5;
  instance = this;
  pwm();
  reset();
  for ( int pch = 0; pch < CHANNELS; pch++ ) {
    channel[pch].logical = channel[pch].pin > -1 ? pch : -1; 
    physical[pch] = pch;
    channel[pch].last_output = -1;
  }
  timer.set( 1000 );
  return *this;          
}

Atm_fc_receiver * Atm_fc_receiver::instance; // Only one instance allowed for now!

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int Atm_fc_receiver::event( int id ) {
  switch ( id ) {
    case EVT_WAIT:
      return channel[0].last_high > 0;
    case EVT_TIMER:
      return timer.expired( this );
    case EVT_CHANGED:
      for ( int i = 0; i <= max_used_channel; i++ ) {
        if ( channel[i].value != channel[i].last_value ) {
          return 1;
        }        
      }
      return 0;
  }
  return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 */

void Atm_fc_receiver::action( int id ) {
  switch ( id ) {
    case ENT_CHANGED:
      for ( int pch = 0; pch <= max_used_channel; pch++ ) {
        if ( channel[pch].logical > -1 && channel[pch].value != channel[pch].last_value ) {
          channel[pch].last_value = channel[pch].value; 
          if ( channel[pch].value < channel[pch].min && channel[pch].value > 800  ) channel[pch].min = channel[pch].value;
          if ( channel[pch].value > channel[pch].max && channel[pch].value < 2200 ) channel[pch].max = channel[pch].value;
          int v = translate( pch );
          if ( v != channel[pch].last_output ) {  
            if ( connector[pch].mode_flags != atm_connector::MODE_NULL ) { // FIXME: is this needed???????
              connector[pch].push( v, channel[pch].logical );
            } else {
              onchange.push( v, channel[pch].logical ); // Only if no specific connector match...                  
            }
            channel[pch].last_output = v;
          }
        }        
      }    
      return;
  }
}

void Atm_fc_receiver::handleInterruptPWM( int pch ) { // pch = physical channel no
  //cli();
  if ( digitalRead( channel[pch].pin ) ) {
    channel[pch].last_high = micros();    
  } else {
    channel[pch].value = micros() - channel[pch].last_high; 
  }
  //sei();  
}

void Atm_fc_receiver::handleInterruptPPM() {
  //cli();
  uint32_t delta = micros() - ppm_last_pulse;
  if ( delta > 4000 ) { // Reset pulse_counter on long gap
    ppm_pulse_counter = 0;
  } else {
    if ( ppm_pulse_counter < CHANNELS ) {
      channel[ppm_pulse_counter].value = delta;
    }        
    ppm_pulse_counter++;
  }
  ppm_last_pulse = micros();
  //sei();  
}

int Atm_fc_receiver::translate( int pch ) { // pch = physical channel no
  int center = channel[pch].max - channel[pch].min / 2;
  if ( channel[pch].value > center + channel[pch].sticky ) { // Upper slope
    if ( channel[pch].value >= channel[pch].max - channel[pch].sticky ) { 
      return 1000; // Top
    } else { 
      return map( channel[pch].value, center + channel[pch].sticky, channel[pch].max, 501, 999 ); 
    }
  } else if ( channel[pch].value <= center - channel[pch].sticky ) { // Lower slope
    if ( channel[pch].value < channel[pch].min + channel[pch].sticky ) { 
      return 0; // Bottom
    } else { 
      return map( channel[pch].value, channel[pch].min, center - channel[pch].sticky, 1, 499 ); 
    }
  }
  return 500; // Dead center
}

Atm_fc_receiver& Atm_fc_receiver::mapping( int lch0, int lch1, int lch2, int lch3, int lch4, int lch5 /* = -1 */ ) {
  channel[0].logical = lch0; 
  channel[1].logical = lch1; 
  channel[2].logical = lch2; 
  channel[3].logical = lch3; 
  channel[4].logical = lch4; 
  channel[5].logical = lch5; 
  if ( lch0 > -1 ) physical[lch0] = 0; 
  if ( lch1 > -1 ) physical[lch1] = 1; 
  if ( lch2 > -1 ) physical[lch2] = 2;
  if ( lch3 > -1 ) physical[lch3] = 3; 
  if ( lch4 > -1 ) physical[lch4] = 4; 
  if ( lch5 > -1 ) physical[lch5] = 5; 
  return *this;
}

int Atm_fc_receiver::read( int lch, bool raw /* = 0 */ ) {
  int pch = physical[lch];
  if ( raw ) {
    return channel[pch].value;
  } else {
    channel[pch].last_output = translate( pch );
    return channel[pch].last_output;
  }
}

Atm_fc_receiver& Atm_fc_receiver::ppm( void ) { // Pulse Position Modulation
  pinMode( channel[0].pin, INPUT_PULLUP );
  if ( channel[0].pin > -1 ) attachInterrupt( channel[0].pin, []() { instance->handleInterruptPPM(); }, RISING );
  if ( channel[1].pin > -1 ) detachInterrupt( channel[1].pin );  
  if ( channel[2].pin > -1 ) detachInterrupt( channel[2].pin );  
  if ( channel[3].pin > -1 ) detachInterrupt( channel[3].pin );  
  if ( channel[4].pin > -1 ) detachInterrupt( channel[4].pin );  
  if ( channel[5].pin > -1 ) detachInterrupt( channel[5].pin );  
  max_used_channel = 5;
  return *this;
}

Atm_fc_receiver& Atm_fc_receiver::pwm( void ) { // Pulse Width Modulation
  for ( int pch = 0; pch < CHANNELS; pch++ ) {
    if ( channel[pch].pin > -1 ) {
      pinMode( channel[pch].pin, INPUT_PULLUP );
      max_used_channel = pch;
    }
  }
  if ( channel[0].pin > -1 ) attachInterrupt( channel[0].pin, []() { instance->handleInterruptPWM( 0 ); }, CHANGE );  
  if ( channel[1].pin > -1 ) attachInterrupt( channel[1].pin, []() { instance->handleInterruptPWM( 1 ); }, CHANGE );  
  if ( channel[2].pin > -1 ) attachInterrupt( channel[2].pin, []() { instance->handleInterruptPWM( 2 ); }, CHANGE );  
  if ( channel[3].pin > -1 ) attachInterrupt( channel[3].pin, []() { instance->handleInterruptPWM( 3 ); }, CHANGE );  
  if ( channel[4].pin > -1 ) attachInterrupt( channel[4].pin, []() { instance->handleInterruptPWM( 4 ); }, CHANGE );  
  if ( channel[5].pin > -1 ) attachInterrupt( channel[5].pin, []() { instance->handleInterruptPWM( 5 ); }, CHANGE );   
  return *this;
}


Atm_fc_receiver& Atm_fc_receiver::sticky( int lch, int value ) {
  channel[physical[lch]].sticky = value;
  return *this;
}

Atm_fc_receiver& Atm_fc_receiver::sticky( int value ) {
  for ( int pch = 0; pch < CHANNELS; pch++ )
    channel[pch].sticky = value;
  return *this;
}

Atm_fc_receiver& Atm_fc_receiver::calibrate( int lch, int min, int max ) {
  channel[physical[lch]].min = min;
  channel[physical[lch]].max = max;
  return *this;
}

Atm_fc_receiver& Atm_fc_receiver::start( void ) {
  trigger( EVT_START );
  return *this;
}

Atm_fc_receiver& Atm_fc_receiver::stop( void ) {
  trigger( EVT_STOP );
  return *this;
}

Atm_fc_receiver& Atm_fc_receiver::toggle( void ) {
  trigger( EVT_TOGGLE );
  return *this;
}

Atm_fc_receiver& Atm_fc_receiver::reset( int lch /* = -1 */ ) {
  if ( lch > -1 ) {
    channel[physical[lch]].min = 1100;
    channel[physical[lch]].max = 1900;
  } else {
    for ( int pch = 0; pch < CHANNELS; pch++ ) {
      channel[pch].min = 1100;
      channel[pch].max = 1900;
    } 
  }
  return *this;
}

int Atm_fc_receiver::minimum( int lch ) {
  return channel[physical[lch]].min;      
}

int Atm_fc_receiver::maximum( int lch ) {
  return channel[physical[lch]].max;      
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

Atm_fc_receiver& Atm_fc_receiver::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int Atm_fc_receiver::state( void ) {
  return Machine::state();
}

Atm_fc_receiver& Atm_fc_receiver::onChange( uint8_t id ) {
  connector[id].mode_flags = atm_connector::MODE_NULL;
  return *this;
}

Atm_fc_receiver& Atm_fc_receiver::onChange( uint8_t id, atm_cb_push_t callback, int idx /* = 0 */ ) {
  connector[id].set( callback, idx );
  return *this;
}

Atm_fc_receiver& Atm_fc_receiver::onChange( uint8_t id, Machine& machine, int event /* = 0 */ ) {
  connector[id].set( &machine, event );
  return *this;
}

Atm_fc_receiver& Atm_fc_receiver::onChange( atm_cb_push_t callback, int idx /* = 0 */ ) {
  onchange.set( callback, idx );
  return *this;
}

Atm_fc_receiver& Atm_fc_receiver::onChange( Machine& machine, int event /* = 0 */ ) {
  onchange.set( &machine, event );
  return *this;
}


/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

Atm_fc_receiver& Atm_fc_receiver::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "RC\0EVT_WAIT\0EVT_TIMER\0EVT_START\0EVT_STOP\0EVT_TOGGLE\0EVT_CHANGED\0ELSE\0IDLE\0WAIT\0PAUSE\0RUN\0CHANGED" );
  return *this;
}



