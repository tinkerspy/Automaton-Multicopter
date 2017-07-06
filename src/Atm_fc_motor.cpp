#include "Atm_fc_motor.h"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

 // Add mapping table for each channnel (1000 * 2 bytes = 2 Kb), pointer per channel (response curve)
 // Keep the state machine to allow ramping...! (motor_cur_speed -> motor_set_speed in stappen. (delta = motor_ramp)

Atm_fc_motor& Atm_fc_motor::begin( int p, int frequency /* = 50 */ ) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*          ON_ENTER    ON_LOOP  ON_EXIT  ELSE */
    /*  IDLE */       -1, ATM_SLEEP,      -1,   -1,
  };
  // clang-format on
  //Machine::begin( state_table, ELSE ); // For now!!!
  motor_pin = p;
  pinMode( motor_pin, OUTPUT );
  analogWriteFrequency( motor_pin, frequency );
  analogWriteResolution( 16 ); // Global effect!
  analogWrite( motor_pin, 0 );
  pwm1000width = ( frequency / 50 ) * PWM_50HZ_1000US;
  return *this;          
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int Atm_fc_motor::event( int id ) {
  switch ( id ) {
  }
  return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 */

void Atm_fc_motor::action( int id ) {
  switch ( id ) {
  }
}

Atm_fc_motor& Atm_fc_motor::speed( int v ) {
  motor_cur_speed = constrain( v, 0, 1000 );
  analogWrite( motor_pin, map( motor_cur_speed, 0, 1000, pwm1000width, pwm1000width * 2  ) );
  return *this;
}

int Atm_fc_motor::speed() {
  return motor_cur_speed;
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

Atm_fc_motor& Atm_fc_motor::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int Atm_fc_motor::state( void ) {
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

Atm_fc_motor& Atm_fc_motor::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "MOTORS\0ELSE\0IDLE" );
  return *this;
}



