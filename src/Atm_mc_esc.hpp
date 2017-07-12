#pragma once

#include <Automaton.h>
#include <Servo.h>
 
#define PWM_50HZ_1000US 3277

// __MKL26Z64__  Teensy LC
// __MK20DX256__ Teensy 3.1/3.2
// __MK62FX512__ Teensy 3.5
// __MK66FX1M0__ Teensy 3.6

// Source: https://github.com/FortySevenEffects/arduino_midi_library/issues/65

#if defined(__MKL26Z64__) || defined(__MK20DX256__) || defined(__MK62FX512__) || defined(__MK66FX1M0__ )
#define TEENSY_HW_PWM
#endif

class Atm_mc_esc {

 public:
  Atm_mc_esc& begin(  int p, int frequency = -1 ); 
  Atm_mc_esc& speed( int v );
  Atm_mc_esc& range( int min_range, int max_range );
  int speed( );
 private:
  Servo servo;  
  int16_t min_range, max_range;
  
};


/*

class Atm_mc_esc {

 public:
   Atm_mc_esc& begin(  int p, int frequency = -1 ); // Stick to multiples of 50, max 400Hz (-1 use Servo library)
  Atm_mc_esc& speed( int v );
  int speed( );
 private:
#ifdef TEENSY_HW_PWM
  int motor_cur_speed;
  int motor_pin; 
  int pwm1000width;
#endif  
  Servo servo;  
  bool servo_mode;
 
};

*/