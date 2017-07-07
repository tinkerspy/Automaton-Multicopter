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

#ifdef __MKL26Z64__
yep Teensy LC
#endif

#ifdef __MK20DX256__
yep Teensy 3.1/3.2
#endif

#ifdef __MK62FX512__
yep Teensy 3.5
#endif

#ifdef __MK66FX1M0__
yep Teensy 3.6
#endif

#ifdef TEENSY_HW_PWM
yep Teensy HW PWM
#endif

class Atm_mc_esc: public Machine {

 public:
 
  enum { IDLE }; // STATES
  enum { ELSE }; // EVENTS
  Atm_mc_esc( void ) : Machine() {};
  Atm_mc_esc& begin(  int p, int frequency = -1 ); // Stick to multiples of 50, max 400Hz (-1 use Servo library)
  Atm_mc_esc& trace( Stream & stream );
  Atm_mc_esc& trigger( int event );
  Atm_mc_esc& speed( int v );
  int speed( );
  int state( void );
  int motor_pin; 
  int motor_cur_speed;

 private:
  enum {  }; // ACTIONS
  int event( int id ); 
  void action( int id ); 
  int pwm1000width;
  Servo servo;  
  int servo_mode;
 
};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_mc_esc">
    <states>
      <IDLE index="0" sleep="1">
      </IDLE>
    </states>
    <events>
    </events>
    <connectors>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/

