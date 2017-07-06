#pragma once

#include <Automaton.h>
#include <Servo.h>
 
#define PWM_50HZ_1000US 3277

class Atm_fc_motor: public Machine {

 public:
 
  enum { IDLE }; // STATES
  enum { ELSE }; // EVENTS
  Atm_fc_motor( void ) : Machine() {};
  Atm_fc_motor& begin(  int p, int frequency = -1 ); // Stick to multiples of 50, max 400Hz (-1 use Servo library)
  Atm_fc_motor& trace( Stream & stream );
  Atm_fc_motor& trigger( int event );
  Atm_fc_motor& speed( int v );
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
  <machine name="Atm_fc_motor">
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

