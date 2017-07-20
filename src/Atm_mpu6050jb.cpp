#include <Atm_mpu6050jb.hpp>

// FIXME: De YAW rate heeft een grotere range dan -9000..9000 maar wordt hier afgeknepen!
// Waarsch 180000..180000 (buiten int16_t range)
// YAW rate waarden kloppen dus waarsch van geen kant!

Atm_mpu6050jb& Atm_mpu6050jb::begin( int sample_rate_ms ) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*               ON_ENTER    ON_LOOP   ON_EXIT  EVT_SAMPLE EVT_CHANGE  EVT_TIMER  EVT_COUNTER  EVT_START  EVT_STOP  EVT_INITDONE    ELSE */
    /*    IDLE */          -1, ATM_SLEEP,       -1,         -1,        -1,        -1,          -1,      INIT,       -1,           -1,     -1,
    /*    INIT */    ENT_INIT,        -1,       -1,         -1,        -1,      INIT,         CAL,        -1,     IDLE,           -1,     -1,
    /*     CAL */     ENT_CAL,        -1,       -1,         -1,        -1,        -1,          -1,        -1,     IDLE,           -1, SAMPLE,
    /*  SAMPLE */  ENT_SAMPLE,        -1,       -1,         -1,   CHANGED,    SAMPLE,          -1,        -1,     IDLE,           -1,     -1,
    /* CHANGED */ ENT_CHANGED,        -1,       -1,         -1,        -1,        -1,          -1,        -1,     IDLE,           -1, SAMPLE,
  };  
  // clang-format on
  Machine::begin( state_table, ELSE );
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  setup_mpu_6050_registers();
  timer.set( sample_rate_ms );
  init_counter.set( 2000 );

  mapping( YAW, PITCH, ROLL );
  range( -90, +90 );
  return *this;          
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int Atm_mpu6050jb::event( int id ) {
  switch ( id ) {
    case EVT_CHANGE:
      return axis[0].value != axis[0].last_value || axis[1].value != axis[1].last_value || axis[2].value != axis[2].last_value;
    case EVT_TIMER:      
      return timer.expired( this );
    case EVT_COUNTER:
      return init_counter.expired();
    case EVT_SAMPLE:
      return 1;
    case EVT_INITDONE:
      return 1;
  }
  return 0;
}


enum { X, Y, Z };
enum { P, R };

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 *
 * Available connectors:
 *   push( connectors, ON_CHANGE, 0, <v>, <up> );
 */

void Atm_mpu6050jb::action( int id ) {
  float angle_pitch_acc, angle_roll_acc;
  int32_t acc_total_vector;
  switch ( id ) {
    case ENT_INIT:
      read_mpu_6050_data();              //Read the raw acc and gyro data from the MPU-6050
      axis[X].gyro_cal += axis[X].gyro;  //Add the gyro x-axis offset to the gyro_x_cal variable
      axis[Y].gyro_cal += axis[Y].gyro;  //Add the gyro y-axis offset to the gyro_y_cal variable
      axis[Z].gyro_cal += axis[Z].gyro;  //Add the gyro z-axis offset to the gyro_z_cal variable
      init_counter.decrement();
      return;
    case ENT_CAL:
      axis[X].gyro_cal /= 2000; //Divide the gyro_x_cal variable by 2000 to get the avarage offset
      axis[Y].gyro_cal /= 2000; //Divide the gyro_y_cal variable by 2000 to get the avarage offset
      axis[Z].gyro_cal /= 2000; //Divide the gyro_z_cal variable by 2000 to get the avarage offset
      
      Serial.print( "CAL: " );
      Serial.print( axis[X].gyro_cal );
      Serial.print( "\t" );
      Serial.print( axis[Y].gyro_cal );
      Serial.print( "\t" );
      Serial.print( axis[Z].gyro_cal );
      Serial.print( "\n" );
      
      return;
    case ENT_SAMPLE:
      read_mpu_6050_data();    //Read the raw acc and gyro data from the MPU-6050

      axis[X].gyro -= axis[X].gyro_cal;   //Subtract the offset calibration value from the raw gyro_x value
      axis[Y].gyro -= axis[Y].gyro_cal;   //Subtract the offset calibration value from the raw gyro_y value
      axis[Z].gyro -= axis[Z].gyro_cal;   //Subtract the offset calibration value from the raw gyro_z value
      
      //Gyro angle calculations
      //0.0000611 = 1 / (250Hz / 65.5)
      axis[P].angle += axis[X].gyro * 0.0000611; //Calculate the traveled pitch angle and add this to the angle_pitch variable
      axis[R].angle += axis[Y].gyro * 0.0000611;  //Calculate the traveled roll angle and add this to the angle_roll variable
      
      //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
      axis[P].angle += axis[R].angle * sin(axis[Z].gyro * 0.000001066); //If the IMU has yawed transfer the roll angle to the pitch angel
      axis[R].angle -= axis[P].angle * sin(axis[Z].gyro * 0.000001066); //If the IMU has yawed transfer the pitch angle to the roll angel
      
      //Accelerometer angle calculations
      acc_total_vector = sqrt((axis[X].acc*axis[X].acc)+(axis[Y].acc*axis[Y].acc)+(axis[Z].acc*axis[Z].acc));  //Calculate the total accelerometer vector
      
      //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
      angle_pitch_acc = asin((float)axis[1].acc/acc_total_vector)* 57.296;       //Calculate the pitch angle
      angle_roll_acc = asin((float)axis[0].acc/acc_total_vector)* -57.296;       //Calculate the roll angle
      
      //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
      angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
      angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

      if(set_gyro_angles){                                                 //If the IMU is already started
        axis[P].angle = axis[P].angle * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        axis[R].angle = axis[R].angle * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
      }
      else{                                                                //At first start
        axis[P].angle = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
        axis[R].angle = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
        set_gyro_angles = true;                                            //Set the IMU started flag
      }
      
      //To dampen the pitch and roll angles a complementary filter is used
      axis[P].angle_output = axis[P].angle_output * 0.9 + axis[P].angle * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
      axis[R].angle_output = axis[R].angle_output * 0.9 + axis[R].angle * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
      axis[P].value = axis[P].angle_output * 100;
      axis[R].value = axis[R].angle_output * 100;
      
      if ( ++cnt % 250 == 0 ) {
        Serial.print( millis() );
        Serial.print( " AG: P=");
        Serial.print( axis[P].value );
        Serial.print( " R=");
        Serial.println( axis[R].value );
      }
      return;
    case ENT_CHANGED:
      for ( int i = YAW; i < ROLL + 1; i++ ) {
        int v = read( i );
        if ( v != axis[i].last_output && !enable_stabilize && !axis[i].master ) {
          push( connectors, ON_CHANGE, axis[i].logical, v, axis[i].logical );    
          axis[i].last_output = v;
          axis[i].last_value = axis[i].value;
        }
      }
      return;
  }
}

// FIXME: De YAW rate heeft een grotere range dan -9000..9000 maar wordt hier afgeknepen!

int Atm_mpu6050jb::read( int ypr ) {
  ypr = physical[ypr];
  int v = axis[ypr].value; // + axis[ypr].offset;
  if ( axis[ypr].reverse ) v = v * -1;
  return map( constrain( v, -9000, 9000 ), -9000, 9000, axis[ypr].min_out, axis[ypr].max_out );    
}


int Atm_mpu6050jb::rate( void ) {
  return rate_fin_counter;
}

Atm_mpu6050jb& Atm_mpu6050jb::stabilize( uint16_t win_size /* = 5 */, uint16_t win_millis /* = 5000 */ ) {
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
/*
Atm_mpu6050jb& Atm_mpu6050jb::calibrate( int ypr, int v ) {
  axis[physical[ypr]].offset = v;
  return *this;
}

Atm_mpu6050jb& Atm_mpu6050jb::calibrate( int ypr ) {
  axis[physical[ypr]].offset = - axis[physical[ypr]].value;
  axis[physical[ypr]].last_value = 0xFF; // Force update event
  axis[physical[ypr]].last_output = 0xFF; 
  axis[physical[ypr]].value = 0xFF; 
  return *this;
}
*/

Atm_mpu6050jb& Atm_mpu6050jb::master( int ypr, bool master /* = true */ ) {
  axis[physical[ypr]].master = master;
  return *this;
}

Atm_mpu6050jb& Atm_mpu6050jb::master( bool master /* = true */ ) {
  for ( int ax = YAW; ax < ROLL + 1; ax++ ) {
    axis[physical[ax]].master = master;
  }
  return *this;
}

Atm_mpu6050jb& Atm_mpu6050jb::range( int ypr, int toLow, int toHigh ) {
  ypr = physical[ypr];
  axis[ypr].min_out = toLow;
  axis[ypr].max_out = toHigh;
  return *this;
}

Atm_mpu6050jb& Atm_mpu6050jb::range( int toLow, int toHigh ) {
  for ( int i = YAW; i < ROLL + 1; i++ ) {
    range( i, toLow, toHigh );
  }
  return *this;
}

Atm_mpu6050jb& Atm_mpu6050jb::angle( int max_angle ) {
  for ( int i = YAW; i < ROLL + 1; i++ ) {
    angle( i, max_angle );
  }
  return *this;
}

Atm_mpu6050jb& Atm_mpu6050jb::angle( int ypr, int max_angle ) {
  int outside = ( 90 - max_angle ) * ( 1001 / ( max_angle * 2 ) );
  range( ypr, 0 - outside, 1000 + outside );
  return *this;
}

Atm_mpu6050jb& Atm_mpu6050jb::mapping( int axis0, int axis1, int axis2 ) {
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


void Atm_mpu6050jb::setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}


void Atm_mpu6050jb::read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  axis[0].acc_lo = Wire.read();
  axis[0].acc_hi = Wire.read();
  axis[1].acc_lo = Wire.read();
  axis[1].acc_hi = Wire.read();
  axis[2].acc_lo = Wire.read();
  axis[2].acc_hi = Wire.read();
  temperature = Wire.read() << 8 | Wire.read();                        //Add the low and high byte to the temperature variable
  axis[0].gyro_lo = Wire.read();
  axis[0].gyro_hi = Wire.read();
  axis[1].gyro_lo = Wire.read();
  axis[1].gyro_hi = Wire.read();
  axis[2].gyro_lo = Wire.read();
  axis[2].gyro_hi = Wire.read();

}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

Atm_mpu6050jb& Atm_mpu6050jb::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int Atm_mpu6050jb::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */

Atm_mpu6050jb& Atm_mpu6050jb::start() {
  trigger( EVT_START );
  return *this;
}

Atm_mpu6050jb& Atm_mpu6050jb::stop() {
  trigger( EVT_STOP );
  return *this;
}

/*
 * onChange() push connector variants ( slots 3, autostore 0, broadcast 0 )
 */

Atm_mpu6050jb& Atm_mpu6050jb::onChange( Machine& machine, int event ) {
  onPush( connectors, ON_CHANGE, 0, 3, 1, machine, event );
  return *this;
}

Atm_mpu6050jb& Atm_mpu6050jb::onChange( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_CHANGE, 0, 3, 1, callback, idx );
  return *this;
}

Atm_mpu6050jb& Atm_mpu6050jb::onChange( int sub, Machine& machine, int event ) {
  onPush( connectors, ON_CHANGE, sub, 3, 0, machine, event );
  return *this;
}

Atm_mpu6050jb& Atm_mpu6050jb::onChange( int sub, atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_CHANGE, sub, 3, 0, callback, idx );
  return *this;
}

/*
 * onStabilize() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

Atm_mpu6050jb& Atm_mpu6050jb::onStabilize( Machine& machine, int event ) {
  onPush( connectors, ON_STABILIZE, 0, 1, 1, machine, event );
  return *this;
}

Atm_mpu6050jb& Atm_mpu6050jb::onStabilize( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_STABILIZE, 0, 1, 1, callback, idx );
  return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

Atm_mpu6050jb& Atm_mpu6050jb::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "MPU6050\0EVT_SAMPLE\0EVT_CHANGE\0EVT_TIMER\0EVT_COUNTER\0EVT_START\0EVT_STOP\0EVT_INITDONE\0ELSE\0IDLE\0INIT\0CAL\0SAMPLE\0CHANGED" );
  return *this;
}



