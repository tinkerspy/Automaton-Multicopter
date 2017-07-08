#include "Atm_mc_esc.hpp"

// Uses the built-in hardware PWM on teensy
// Uses the Servo library on other platforms
 
Atm_mc_esc& Atm_mc_esc::begin( int p, int frequency /* = 50 */ ) {
  motor_pin = p;
  pinMode( motor_pin, OUTPUT );
  if ( frequency == -1 ) {
    servo.attach( p );
    servo_mode = 1;
  } else {
#ifdef TEENSY_HW_PWM
    analogWriteFrequency( motor_pin, frequency );
    analogWriteResolution( 16 ); // Global effect!
    pwm1000width = ( frequency / 50 ) * PWM_50HZ_1000US;
#endif    
    analogWrite( motor_pin, 0 );
    servo_mode = 0;
  }
  return *this;          
}


Atm_mc_esc& Atm_mc_esc::speed( int v ) {
  motor_cur_speed = constrain( v, 0, 1000 );
  if ( servo_mode ) {
    servo.writeMicroseconds( map( motor_cur_speed, 0, 1000, 1000, 2000  ) );
  } else {
#ifdef TEENSY_HW_PWM
    analogWrite( motor_pin, map( motor_cur_speed, 0, 1000, pwm1000width, pwm1000width * 2  ) );
#endif
    }
  return *this;
}

int Atm_mc_esc::speed() {
  return motor_cur_speed;
}

