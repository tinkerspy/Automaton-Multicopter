#include <Automaton_Multicopter.h>

enum CONTROLS { YAW, PITCH, ROLL, THROTTLE, AUX1, AUX2 };

int pin_rx[] = { 23, 22, 21, 20 };
int pin_motor[] = { 3, 4, 6, 9 };

Atm_mc_receiver rx;
Atm_mc_mixer mixer;
Atm_mc_esc motor[4];
Atm_pid pid[4];
Atm_mpu6050 mpu;

void pid_callback( int idx, int value, int up ) {
  mixer.set( idx, value ); 
}
   
void setup() {

  for ( int i = 0; i < 4; i++ ) 
    motor[i].begin( pin_motor[i], 100 ).speed( 0 );

  rx.begin( pin_rx[0], pin_rx[1], pin_rx[2], pin_rx[3] )
    .mapping( YAW, THROTTLE, PITCH, ROLL )
    .calibrate( THROTTLE, 993, 1988 )
    .calibrate(      YAW, 996, 1983 )
    .calibrate(    PITCH, 997, 1988 )
    .calibrate(     ROLL, 998, 1988 )
    .sticky( 20 )
    .onChange( [] ( int idx, int value, int axis ) {  
      pid[axis].sp( value );
    })
    .start();

  mixer.begin( mixer.CFG_MIXER_QUADX )
   .onChange( [] ( int idx, int value, int ch ) {
     motor[ch].speed( value );
   });

  mpu.begin( 100 )
    .mapping( YAW, ROLL | mpu.REVERSE, PITCH )
    .angle( 45 ) 
    .onChange( [] ( int idx, int value, int axis ) {
      pid[axis].pv( value );
    })
    .start();

  pid[     YAW].begin( 100, 0.2, 2.0, 0.0, 100  ).onChange( pid_callback,      YAW );
  pid[   PITCH].begin( 100, 0.2, 2.0, 0.0, 100  ).onChange( pid_callback,    PITCH ).start();
  pid[    ROLL].begin( 100, 0.2, 2.0, 0.0, 100  ).onChange( pid_callback,     ROLL ).start();
  pid[THROTTLE].begin( 100, 0.2, 2.0, 0.0, 100  ).onChange( pid_callback, THROTTLE ); // Pass through

  mixer.start();
}

void loop() {
  automaton.run();
}
