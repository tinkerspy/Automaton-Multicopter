#include "Atm_mc_accelgyro.hpp"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

Atm_mc_accelgyro& Atm_mc_accelgyro::begin( IMU & imu, uint32_t sample_rate_us ) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*               ON_ENTER    ON_LOOP  ON_EXIT  EVT_TIMER  EVT_READ  EVT_START  EVT_STOP  ELSE */
    /*    IDLE */    ENT_IDLE, ATM_SLEEP,      -1,        -1,       -1,       RUN,       -1,   -1,
    /*     RUN */     ENT_RUN,        -1,      -1,    SAMPLE,       -1,        -1,     IDLE,   -1,
    /*  SAMPLE */  ENT_SAMPLE,        -1,      -1,        -1,  READING,        -1,     IDLE,   -1,
    /* READING */ ENT_READING,        -1,      -1,        -1,       -1,        -1,     IDLE,  RUN,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  microtimer_value = sample_rate_us;
  this->imu = &imu;
  this->imu->init( sample_rate_us );
  inputMapping( YAW, PITCH, ROLL );
  outputRange( -90, +90 );
  return *this;          
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int Atm_mc_accelgyro::event( int id ) {
  switch ( id ) {
    case EVT_TIMER:
      if ( microtimer == ATM_TIMER_OFF ? 0 : micros() - microtimer >= microtimer_value && imu->lockChannel( true ) ) {
        microtimer = micros();
        return 1;
      } else {
        return 0;
      }
    case EVT_READ:
      return imu->sampleReady();
    }
  return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 *
 * Available connectors:
 *   push( connectors, ON_CHANGE, <sub>, <v>, <up> );
 *   push( connectors, ON_UPDATE, 0, <v>, <up> );
 */

void Atm_mc_accelgyro::action( int id ) {
  switch ( id ) {
    case ENT_IDLE:
      return;
    case ENT_RUN:    
      return;
    case ENT_SAMPLE:
      if ( imu->sampleAvailable() ) {
        push( connectors, ON_SAMPLE, 0, 0, 0 );
        imu->requestSample();
      }
      return;
    case ENT_READING:
      imu->readSample();
      imu->lockChannel( false );
      imu->computeAngles();
      axis[0].value = imu->angleZ() * 100;
      axis[1].value = imu->angleX() * 100;
      axis[2].value = imu->angleY() * 100;
      push( connectors, ON_UPDATE, 0, read( PITCH ), read( ROLL ) );
      for ( int ax = 0; ax < 3; ax++ ) {
        if ( axis[ax].value != axis[ax].last_value ) {
          push( connectors, ON_CHANGE, ax, read( ax ), 0 );
          axis[ax].last_value = axis[ax].value;
        }
      }
      return;
  }
}

Atm_mc_accelgyro& Atm_mc_accelgyro::inputMapping( int axis0, int axis1, int axis2 ) {
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

Atm_mc_accelgyro& Atm_mc_accelgyro::outputRange( int ypr, int toLow, int toHigh ) {
  ypr = physical[ypr];
  axis[ypr].min_out = toLow;
  axis[ypr].max_out = toHigh;
  return *this;
}

Atm_mc_accelgyro& Atm_mc_accelgyro::outputRange( int toLow, int toHigh ) {
  for ( int i = YAW; i < ROLL + 1; i++ ) {
    outputRange( i, toLow, toHigh );
  }
  return *this;
}

Atm_mc_accelgyro& Atm_mc_accelgyro::angle( int max_angle ) {
  for ( int i = YAW; i < ROLL + 1; i++ ) {
    angle( i, max_angle );
  }
  return *this;
}

Atm_mc_accelgyro& Atm_mc_accelgyro::angle( int ypr, int max_angle ) {
  int outside = ( 90 - max_angle ) * ( 1001 / ( max_angle * 2 ) );
  outputRange( ypr, 0 - outside, 1000 + outside );
  return *this;
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

Atm_mc_accelgyro& Atm_mc_accelgyro::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int Atm_mc_accelgyro::state( void ) {
  return Machine::state();
}

int Atm_mc_accelgyro::read( int ypr ) {
  ypr = physical[ypr];
  int v = axis[ypr].value + axis[ypr].offset;
  if ( axis[ypr].reverse ) v = v * -1;
  return map( constrain( v, -9000, 9000 ), -9000, 9000, axis[ypr].min_out, axis[ypr].max_out );    
}

/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */

Atm_mc_accelgyro& Atm_mc_accelgyro::start() {
  trigger( EVT_START );
  return *this;
}

Atm_mc_accelgyro& Atm_mc_accelgyro::stop() {
  trigger( EVT_STOP );
  return *this;
}

/*
 * onChange() push connector variants ( slots 3, autostore 0, broadcast 0 )
 */

Atm_mc_accelgyro& Atm_mc_accelgyro::onChange( Machine& machine, int event ) {
  onPush( connectors, ON_CHANGE, 0, 3, 1, machine, event );
  return *this;
}

Atm_mc_accelgyro& Atm_mc_accelgyro::onChange( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_CHANGE, 0, 3, 1, callback, idx );
  return *this;
}

Atm_mc_accelgyro& Atm_mc_accelgyro::onChange( int sub, Machine& machine, int event ) {
  onPush( connectors, ON_CHANGE, sub, 3, 0, machine, event );
  return *this;
}

Atm_mc_accelgyro& Atm_mc_accelgyro::onChange( int sub, atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_CHANGE, sub, 3, 0, callback, idx );
  return *this;
}

/*
 * onUpdate() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

Atm_mc_accelgyro& Atm_mc_accelgyro::onSample( Machine& machine, int event ) {
  onPush( connectors, ON_SAMPLE, 0, 1, 1, machine, event );
  return *this;
}

Atm_mc_accelgyro& Atm_mc_accelgyro::onSample( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_SAMPLE, 0, 1, 1, callback, idx );
  return *this;
}

Atm_mc_accelgyro& Atm_mc_accelgyro::onUpdate( Machine& machine, int event ) {
  onPush( connectors, ON_UPDATE, 0, 1, 1, machine, event );
  return *this;
}

Atm_mc_accelgyro& Atm_mc_accelgyro::onUpdate( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_UPDATE, 0, 1, 1, callback, idx );
  return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

Atm_mc_accelgyro& Atm_mc_accelgyro::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "ACCELGYRO\0EVT_TIMER\0EVT_READ\0EVT_START\0EVT_STOP\0ELSE\0IDLE\0RUN\0SAMPLE\0READING" );
  return *this;
}



