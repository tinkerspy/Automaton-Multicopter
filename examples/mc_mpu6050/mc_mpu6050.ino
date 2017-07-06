#include <Automaton_Multicopter.h>

// WARNING: quality wiring is critical

enum{ YAW, PITCH, ROLL };

Atm_timer timer;
Atm_mpu6050 mpu;

void timer_callback( int idx, int v, int cnt ) {
    Serial.print( cnt );
    Serial.print( "\tRead:  Y " );
    Serial.print( mpu.read( YAW ) );
    Serial.print( "\tP " );
    Serial.print( mpu.read( PITCH ) );
    Serial.print( "\tR " );
    Serial.print( mpu.read( ROLL ) );
    Serial.print( "\t\r\n" );
    if ( cnt % 10 == 0 ) {
      Serial.println( "--------------------------------------------------------------------" );
    }
}

void setup() {
  Serial.begin( 9600 );
  delay( 1000 );
  Serial.println( "Start MPU605 test" );

  mpu.begin( 40 )
    .mapping( YAW, ROLL | mpu.REVERSE, PITCH ) // Set high bit to reverse
    .range( -9000, 9000 ) // Set range
    .angle( PITCH, 45 ) // Set range with angle for P & R
    .angle( ROLL, 45 )
    .start();
   
  Serial.println( "Start timer" );
  timer.begin( 1000 )
    .onTimer( timer_callback )
    .repeat()
    .start();

}

void loop() {
  automaton.run();
}



