#include "Atm_mc_esc.hpp"

// Uses the built-in hardware PWM on teensy
// Uses the Servo library on other platforms

// Separate code for Teensy & UNO (no extra memory use on UNO possible) drop: servo_mode
 
Atm_mc_esc& Atm_mc_esc::begin( int p, int frequency /* = 50 */ ) {
  if ( frequency == -1 ) {
    servo.attach( p );
    servo_mode = true;
  } else {
#ifdef TEENSY_HW_PWM
    motor_pin = p;
    pinMode( p, OUTPUT );
    analogWriteFrequency( motor_pin, frequency );
    analogWriteResolution( 16 ); // Global effect!
    pwm1000width = ( frequency / 50 ) * PWM_50HZ_1000US;
    analogWrite( motor_pin, 0 );
#endif    
    servo_mode = false;
  }
  speed( 0 );
  return *this;          
}


Atm_mc_esc& Atm_mc_esc::speed( int v ) {
  int motor_cur_speed = constrain( v, 0, 1000 );
  if ( servo_mode ) {
    servo.writeMicroseconds( motor_cur_speed + 1000 );
  } else {
#ifdef TEENSY_HW_PWM
    analogWrite( motor_pin, map( motor_cur_speed, 0, 1000, pwm1000width, pwm1000width * 2  ) );
#endif
    }
  return *this;
}

int Atm_mc_esc::speed() {
#ifdef TEENSY_HW_PWM
  return motor_cur_speed;
#else
  return servo.readMicroseconds();
#endif
}

