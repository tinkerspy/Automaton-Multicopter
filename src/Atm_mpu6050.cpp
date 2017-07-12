#include "Atm_mpu6050.hpp"
#include "MPU6050_6Axis_MotionApps20.h"

// FIXME: De YAW rate heeft een grotere range dan -9000..9000 maar wordt hier afgeknepen!
// Waarsch 180000..180000 (buiten int16_t range)
// YAW rate waarden kloppen dus waarsch van geen kant!

Atm_mpu6050& Atm_mpu6050::begin( int sample_rate_ms ) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*            ON_ENTER    ON_LOOP  ON_EXIT    EVT_SAMPLE  EVT_CHANGE  EVT_TIMER  EVT_START  EVT_STOP  EVT_INITDONE  ELSE */
    /*    IDLE */          -1, ATM_SLEEP,      -1,         -1,        -1,        -1,      INIT,       -1,           -1,   -1,
    /*    INIT */    ENT_INIT,        -1,      -1,         -1,        -1,        -1,        -1,     IDLE,          RUN,   -1,
    /*     RUN */     ENT_RUN,        -1,      -1,         -1,        -1,     CHECK,        -1,     IDLE,           -1,   -1,
    /*   CHECK */   ENT_CHECK,        -1,      -1,     SAMPLE,        -1,        -1,        -1,     IDLE,           -1,  RUN,
    /*  SAMPLE */  ENT_SAMPLE,        -1,      -1,         -1,   CHANGED,        -1,        -1,     IDLE,           -1,  RUN,
    /* CHANGED */ ENT_CHANGED,        -1,      -1,         -1,        -1,        -1,        -1,     IDLE,           -1,  RUN,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  Wire.begin();
  Wire.setClock(500000L);
//  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu6050.initialize();
  int devStatus = mpu6050.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu6050.setXGyroOffset(220);
  mpu6050.setYGyroOffset(76);
  mpu6050.setZGyroOffset(-85);
  mpu6050.setZAccelOffset(1688); // 1688 factory default for my test chip
  if (devStatus == 0) {
      mpu6050.setDMPEnabled(true);
      packetSize = mpu6050.dmpGetFIFOPacketSize();
  } 
  timer.set( sample_rate_ms );
  // Defaults
  mapping( YAW, PITCH, ROLL );
  range( -90, +90 );
  stabilize( 5, 5000 );
  return *this;          
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int Atm_mpu6050::event( int id ) {
  switch ( id ) {
    case EVT_CHANGE:
      return axis[0].value != axis[0].last_value || axis[1].value != axis[1].last_value || axis[2].value != axis[2].last_value;
    case EVT_TIMER:      
      return timer.expired( this );
    case EVT_SAMPLE:
      fifoCount = mpu6050.getFIFOCount(); 
      return fifoCount >= packetSize;
    case EVT_INITDONE:
      return 1;
  }
  return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 *
 * Available connectors:
 *   push( connectors, ON_CHANGE, 0, <v>, <up> );
 */

void Atm_mpu6050::action( int id ) {
  Quaternion q;
  VectorFloat gravity;
  uint8_t fifoBuffer[42];
  float tmp[3];
  switch ( id ) {
    case ENT_INIT:
      return;
    case ENT_SAMPLE:
      mpu6050.getFIFOBytes( fifoBuffer, packetSize );
      fifoCount -= packetSize; 
      mpu6050.dmpGetQuaternion( &q, fifoBuffer );
      mpu6050.dmpGetGravity( &gravity, &q );
      mpu6050.dmpGetYawPitchRoll( tmp, &q, &gravity );
      for ( int ax = 0; ax < 3; ax ++ ) {
        byte cur_second =  ( millis() / rate_millis ) % 60;
        int16_t v = round( ( tmp[ax] * 180.0/M_PI ) * 100.0 );
        if ( cur_second != rate_cur_second ) {
          rate_fin_counter = rate_cur_counter;
          rate_cur_second = cur_second;
          rate_cur_counter = 0;
        }
        if ( abs( axis[ax].rate_pos - v ) > rate_win ) {
          axis[ax].rate_pos = v;
          rate_cur_counter++;
        }          
        axis[ax].value = v;
      }
      if ( enable_stabilize && rate_fin_counter  == 0 ) {
        push( connectors, ON_STABILIZE, 0, 0, 0 );    
        enable_stabilize = false;
      } 
      return;
    case ENT_CHANGED:
      for ( int i = YAW; i < ROLL + 1; i++ ) {
        int v = read( i );
        if ( v != axis[i].last_output ) {
          push( connectors, ON_CHANGE, axis[i].logical, v, axis[i].logical );    
          axis[i].last_output = v;
          axis[i].last_value = axis[i].value;
        }
      }
      return;
  }
}

// FIXME: De YAW rate heeft een grotere range dan -9000..9000 maar wordt hier afgeknepen!

int Atm_mpu6050::read( int ypr ) {
  ypr = physical[ypr];
  int v = axis[ypr].value + axis[ypr].offset;
  if ( axis[ypr].reverse ) v = v * -1;
  return map( constrain( v, -9000, 9000 ), -9000, 9000, axis[ypr].min_out, axis[ypr].max_out );    
}


int Atm_mpu6050::rate( void ) {
  return rate_fin_counter;
}

Atm_mpu6050& Atm_mpu6050::stabilize( uint16_t win_size, uint16_t win_millis ) {
  for ( int ax = YAW; ax < ROLL + 1; ax++ ) {
    axis[ax].rate_pos = 0;
  }
  rate_win = win_size;  
  rate_millis = win_millis;  
  rate_cur_counter = 0xFF;
  rate_fin_counter = 0xFF;
  rate_cur_second = ( millis() / 1000 ) % 60;
  enable_stabilize = true;
  return *this;
}

Atm_mpu6050& Atm_mpu6050::calibrate( int ypr, int v ) {
  axis[physical[ypr]].offset = v;
  return *this;
}

Atm_mpu6050& Atm_mpu6050::calibrate( int ypr ) {
  axis[physical[ypr]].offset = - axis[physical[ypr]].value;
  axis[physical[ypr]].last_value = 0;
  return *this;
}

Atm_mpu6050& Atm_mpu6050::range( int ypr, int toLow, int toHigh ) {
  ypr = physical[ypr];
  axis[ypr].min_out = toLow;
  axis[ypr].max_out = toHigh;
  return *this;
}

Atm_mpu6050& Atm_mpu6050::range( int toLow, int toHigh ) {
  for ( int i = YAW; i < ROLL + 1; i++ ) {
    range( i, toLow, toHigh );
  }
  return *this;
}

Atm_mpu6050& Atm_mpu6050::angle( int max_angle ) {
  for ( int i = YAW; i < ROLL + 1; i++ ) {
    angle( i, max_angle );
  }
  return *this;
}

Atm_mpu6050& Atm_mpu6050::angle( int ypr, int max_angle ) {
  int outside = ( 90 - max_angle ) * ( 1001 / ( max_angle * 2 ) );
  range( ypr, 0 - outside, 1000 + outside );
  return *this;
}

Atm_mpu6050& Atm_mpu6050::mapping( int axis0, int axis1, int axis2 ) {
  axis[0].logical = axis0 & ~REVERSE;
  axis[0].reverse = axis0 & REVERSE;
  axis[1].logical = axis1 & ~REVERSE;
  axis[1].reverse = axis1 & REVERSE;
  axis[2].logical = axis2 & ~REVERSE;
  axis[2].reverse = axis2 & REVERSE;
  physical[axis0 & ~REVERSE] = 0;
  physical[axis1 & ~REVERSE] = 1;
  physical[axis2 & ~REVERSE] = 2;
  return *this;  
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

Atm_mpu6050& Atm_mpu6050::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int Atm_mpu6050::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */

Atm_mpu6050& Atm_mpu6050::start() {
  trigger( EVT_START );
  return *this;
}

Atm_mpu6050& Atm_mpu6050::stop() {
  trigger( EVT_STOP );
  return *this;
}

/*
 * onChange() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

 Atm_mpu6050& Atm_mpu6050::onStabilize( Machine& machine, int event ) {
  onPush( connectors, ON_STABILIZE, 0, 1, 1, machine, event );
  return *this;
}

Atm_mpu6050& Atm_mpu6050::onStabilize( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_STABILIZE, 0, 1, 1, callback, idx );
  return *this;
}

 Atm_mpu6050& Atm_mpu6050::onChange( Machine& machine, int event ) {
  onPush( connectors, ON_CHANGE, 0, 3, 1, machine, event );
  return *this;
}

Atm_mpu6050& Atm_mpu6050::onChange( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_CHANGE, 0, 3, 1, callback, idx );
  return *this;
}

Atm_mpu6050& Atm_mpu6050::onChange( int sub, Machine& machine, int event ) {
  onPush( connectors, ON_CHANGE, sub, 3, 0, machine, event );
  return *this;
}

Atm_mpu6050& Atm_mpu6050::onChange( int sub, atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_CHANGE, sub, 3, 0, callback, idx );
  return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

Atm_mpu6050& Atm_mpu6050::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "MPU6050\0EVT_SAMPLE\0EVT_CHANGE\0EVT_TIMER\0EVT_START\0EVT_STOP\0EVT_INITDONE\0ELSE\0IDLE\0INIT\0RUN\0CHECK\0SAMPLE\0CHANGED" );
  return *this;
}



