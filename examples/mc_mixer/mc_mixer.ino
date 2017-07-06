#include <Automaton_Multicopter.h>

enum CONTROLS { YAW, PITCH, ROLL, THROTTLE };

Atm_mc_mixer mixer;
char cmd_buffer[80];
Atm_command cmd;

enum { CMD_Y, CMD_P, CMD_R, CMD_T };
const char cmdlist[] =  "y p r t";

void cmd_callback( int idx, int v, int up ) {
  int v1 = atoi( cmd.arg( 1 ) );
  switch ( v ) {
    case CMD_Y: 
      mixer.set( YAW, v1 );
      return;
    case CMD_P: 
      mixer.set( PITCH, v1 );
      return;
    case CMD_R: 
      mixer.set( ROLL, v1 );
      return;
    case CMD_T: 
      mixer.set( THROTTLE, v1 );
      return;
  }
}

   
void setup() {
  Serial.begin( 9600 );
  delay( 1000 );
  Serial.println( "Initializing mixer" );

  mixer.begin( mixer.CFG_MIXER_QUADX )
   .onChange( [] ( int idx, int value, int ch ) {
     Serial.print( "Set motor " );
     Serial.print( ch );
     Serial.print( " => " );
     Serial.println( value );
   })
   .start();

  cmd.begin( Serial, cmd_buffer, sizeof( cmd_buffer ) )
    .list( cmdlist )
    .onCommand( cmd_callback );

}

void loop() {
  automaton.run();
}



