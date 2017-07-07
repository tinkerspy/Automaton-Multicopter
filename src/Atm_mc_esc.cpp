#include "Atm_mc_esc.hpp"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

// Uses the built-in hardware PWM on teensy
 
 
Atm_mc_esc& Atm_mc_esc::begin( int p, int frequency /* = 50 */ ) {
  // clang-format off
//  const static state_t state_table[] PROGMEM = {
//    /*          ON_ENTER    ON_LOOP  ON_EXIT  ELSE */
//    /*  IDLE */       -1, ATM_SLEEP,      -1,   -1,
//  };
  // clang-format on
  //Machine::begin( state_table, ELSE ); // For now!!!
  motor_pin = p;
  pinMode( motor_pin, OUTPUT );
  if ( frequency == -1 ) {
    servo.attach( p );
    servo_mode = 1;
  } else {
#ifdef TEENSY_HW_PWM
    analogWriteFrequency( motor_pin, frequency );
    analogWriteResolution( 16 ); // Global effect!
#endif    
    analogWrite( motor_pin, 0 );
    servo_mode = 0;
  }
  pwm1000width = ( frequency / 50 ) * PWM_50HZ_1000US;
  return *this;          
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int Atm_mc_esc::event( int id ) {
  switch ( id ) {
  }
  return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 */

void Atm_mc_esc::action( int id ) {
  switch ( id ) {
  }
}

Atm_mc_esc& Atm_mc_esc::speed( int v ) {
  motor_cur_speed = constrain( v, 0, 1000 );
  if ( servo_mode ) {
    servo.writeMicroseconds( map( motor_cur_speed, 0, 1000, 1000, 2000  ) );
  } else {
    analogWrite( motor_pin, map( motor_cur_speed, 0, 1000, pwm1000width, pwm1000width * 2  ) );
  }
  return *this;
}

int Atm_mc_esc::speed() {
  return motor_cur_speed;
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

Atm_mc_esc& Atm_mc_esc::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int Atm_mc_esc::state( void ) {
  return motor_cur_speed;
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

Atm_mc_esc& Atm_mc_esc::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "MOTORS\0ELSE\0IDLE" );
  return *this;
}



