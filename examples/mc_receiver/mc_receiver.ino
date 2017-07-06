#include <Automaton_Multicopter.h>

enum { YAW, PITCH, ROLL, THROTTLE };

Atm_mc_receiver rx;

void setup() {
  Serial.begin( 9600 );
  delay( 1000 );
  Serial.println( "Initializing receiver" );
  delay( 1000 );

  rx.begin( 23, 22, 21, 20 )
    .mapping( YAW, THROTTLE, PITCH, ROLL )
    .calibrate( THROTTLE, 993, 1988 )
    .calibrate(      YAW, 996, 1983 )
    .calibrate(    PITCH, 997, 1988 )
    .calibrate(     ROLL, 998, 1988 )
    .onChange( [] ( int idx, int value, int axis ) {  
      Serial.print( "Receiver output " );
      Serial.print( axis );
      Serial.print( " => " );
      Serial.println( value );
    })
    .sticky( 20 )
    .start();

}


void loop() {
  automaton.run();
}

