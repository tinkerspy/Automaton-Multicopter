#include "Atm_mc_esc.hpp"

// Uses the built-in hardware PWM on teensy
// Uses the Servo library on other platforms

  
#ifndef __AVR_ATmega328P__

// Servo + Fast PWM option on Teensy 3.x/LC platforms.

 
Atm_mc_esc& Atm_mc_esc::begin( int p, int frequency /* = 50 */ ) {
  if ( frequency == -1 ) {
    servo.attach( p );
    servo_mode = true;
  } else {
    motor_pin = p;
    pinMode( p, OUTPUT );
    analogWriteFrequency( motor_pin, frequency );
    analogWriteResolution( 16 ); // Global effect!
    pwm1000width = ( frequency / 50 ) * PWM_50HZ_1000US;
    analogWrite( motor_pin, 0 );
    servo_mode = false;
  }
  speed( 0 );
  enabled = true;
  return *this;          
}

Atm_mc_esc& Atm_mc_esc::speed( int v ) {
  motor_cur_speed = constrain( v, 0, 1000 );
  if ( enabled ) {
    if ( servo_mode ) {
      servo.writeMicroseconds( motor_cur_speed + 1000 );
    } else {
      analogWrite( motor_pin, map( motor_cur_speed, 0, 1000, pwm1000width, pwm1000width * 2  ) );
    }
  }
  return *this;
}

int Atm_mc_esc::speed() {
  return motor_cur_speed;
}

Atm_mc_esc& Atm_mc_esc::enable( bool v ) {
  if ( !v ) speed ( 0 );
  enabled = v;
  return *this;
}

#else

// Basic version with Servo library only
  
Atm_mc_esc& Atm_mc_esc::begin( int p, int frequency /* = 50 */ ) {
  servo.attach( p );
  speed( 0 );
  enabled = true;
  return *this;          
}
 
Atm_mc_esc& Atm_mc_esc::speed( int v ) {
  int motor_cur_speed = map( constrain( v, 0, 1000 ), 0, 1000, 1000, 2000 );
  if ( enabled ) servo.writeMicroseconds( motor_cur_speed );
  return *this;
}

int Atm_mc_esc::speed() {
  return map( servo.readMicroseconds(), 1000, 2000, 0, 1000 );
}

Atm_mc_esc& Atm_mc_esc::enable( bool v ) {
  if ( !v ) speed ( 0 );
  enabled = v;
  return *this;
}

#endif

