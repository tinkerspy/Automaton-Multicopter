#include <Automaton_Multicopter.h>

enum MOTORS { RF, RR, LR, LF };

int pin[4] = { 3, 4, 6, 9 };
int frequency = -1; // -1 = Servo library, 50-450: Teensy HW PWM

Atm_mc_esc motor[4];
char cmd_buffer[80];
Atm_command cmd;
Atm_led led;

enum { CMD_M, CMD_S, CMD_A };
const char cmdlist[] =  "m s a";

void cmd_callback( int idx, int v, int up ) {
  int v1 = atoi( cmd.arg( 1 ) );
  int v2 = atoi( cmd.arg( 2 ) );
  switch ( v ) {
    case CMD_M: 
      Serial.print( "Set " );
      Serial.print( v1 );
      Serial.print( " to " );
      Serial.println( v2 );
      motor[v1].speed( v2 );
      return;
    case CMD_A: 
      Serial.print( "Set all to " );
      Serial.println( v1 );
      motor[0].speed( v1 );
      motor[1].speed( v1 );
      motor[2].speed( v1 );
      motor[3].speed( v1 );
      return;
    case CMD_S: 
      Serial.println( "Reset " );
      motor[0].speed( 0 );
      motor[1].speed( 0 );
      motor[2].speed( 0 );
      motor[3].speed( 0 );
      return;
  }
}

void setup() {
  Serial.begin( 9600 );
  delay( 1000 );
  Serial.println( "Motor test started" );
  Serial.println( "s: stop all motors" );
  Serial.println( "a ###: set all motors to ###" );
  Serial.println( "m [motor] ###: set [motor] to ###" );

  led.begin( 13 ).blink( 50, 1000 ).start();
  
  cmd.begin( Serial, cmd_buffer, sizeof( cmd_buffer ) )
    .list( cmdlist )
    .onCommand( cmd_callback );

  motor[0].begin( pin[0], frequency ).speed( 0 );
  motor[1].begin( pin[1], frequency ).speed( 0 );
  motor[2].begin( pin[2], frequency ).speed( 0 );
  motor[3].begin( pin[3], frequency ).speed( 0 );

}

void loop() {
  automaton.run();
}



