#include "MPU6050cf.hpp"

// Based on code from http://www.pitt.edu/~mpd41/Angle.ino

MPU6050cf::MPU6050cf( int addr ) {
  address = addr; 
}

void MPU6050cf::init( int16_t sample_interval_us ) {
  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz (500kHz or even 600kHz seems to work! (0.49/0.43/0.35 m/s data transfer)
 
  Wire.beginTransmission( address );
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(100); // FIXME kan waarschijnlijk weg!!!
  
  Wire.beginTransmission( address );
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom( address,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //2) calculate pitch and roll
  //float roll = atan2(AcY, AcZ)* DEGREE_CONVERT;
  //float pitch = atan2(-AcX, AcZ)* DEGREE_CONVERT;
  //
  ////3) set the starting angle to this pitch and roll
  //float gyroXangle = roll; // UNUSED VARIABLES!!!!!
  //float gyroYangle = pitch;
  //float compAngleX = roll;
  //float compAngleY = pitch;

  //start the timer
  timer = micros();
}

void MPU6050cf::calibrate( void ) {
  
}
bool MPU6050cf::calibrateDone( void ) {
  return true;
}

bool MPU6050cf::lockChannel( bool lock ) { 
  return true;
}

bool MPU6050cf::sampleAvailable( void ) { 
  return true; 
}

void MPU6050cf::requestSample( void ) {
  Wire.beginTransmission( address );
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
#ifdef __AVR_ATmega328P__
  Wire.requestFrom( address, 14, true );
#else 
  Wire.sendRequest(address,14, I2C_STOP );  // request a total of 14 registers  
#endif  
}

bool MPU6050cf::sampleReady( void ) {
  return Wire.available() >= 14; 
}

void MPU6050cf::readSample( void ) {
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  dt = (float)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a float saved to "dt".
  timer = micros(); //start the timer again so that we can calculate the next dt.
}


float atan2_approximation1(float y, float x)
{
    //http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
    //Volkan SALMA

    const float ONEQTR_PI = M_PI / 4.0;
	const float THRQTR_PI = 3.0 * M_PI / 4.0;
	float r, angle;
	float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition
	if ( x < 0.0f )
	{
		r = (x + abs_y) / (abs_y - x);
		angle = THRQTR_PI;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = ONEQTR_PI;
	}
	angle += (0.1963f * r * r - 0.9817f) * r;
	if ( y < 0.0f )
		return( -angle );     // negate if in quad III or IV
	else
		return( angle );


}

#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
// |error| < 0.005
float atan2_approximation2( float y, float x )
{
	if ( x == 0.0f )
	{
		if ( y > 0.0f ) return PIBY2_FLOAT;
		if ( y == 0.0f ) return 0.0f;
		return -PIBY2_FLOAT;
	}
	float atan;
	float z = y/x;
	if ( fabs( z ) < 1.0f )
	{
		atan = z/(1.0f + 0.28f*z*z);
		if ( x < 0.0f )
		{
			if ( y < 0.0f ) return atan - PI_FLOAT;
			return atan + PI_FLOAT;
		}
	}
	else
	{
		atan = PIBY2_FLOAT - z/(z*z + 0.28f);
		if ( y < 0.0f ) return atan - PI_FLOAT;
	}
	return atan;
}


void MPU6050cf::computeAngles( void ) {
 
  //the next two lines calculate the orientation of the accelerometer relative to the earth and convert the output of atan2 from radians to degrees
  //We will use this data to correct any cumulative errors in the orientation that the gyroscope develops.
  float roll = atan2(AcY, AcZ)* DEGREE_CONVERT; // Slow: ~400 us on a Teensy LC!
  float pitch = atan2(-AcX, AcZ)* DEGREE_CONVERT;

  //float roll = atan2_approximation1(AcY, AcZ)* DEGREE_CONVERT; // These are much faster, but good enough??
  //float pitch = atan2_approximation1(-AcX, AcZ)* DEGREE_CONVERT;

  //float roll = atan2_approximation2(AcY, AcZ)* DEGREE_CONVERT;
  //float pitch = atan2_approximation2(-AcX, AcZ)* DEGREE_CONVERT;

 //The gyroscope outputs angular velocities.  To convert these velocities from the raw data to deg/second, divide by 131.  
  //Notice, we're dividing by a float "131.0" instead of the int 131.
  float gyroXrate = GyX/131.0;
  float gyroYrate = GyY/131.0;

  //THE COMPLEMENTARY FILTER
  //This filter calculates the angle based MOSTLY on integrating the angular velocity to an angular displacement.
  //dt, recall, is the time between gathering data from the MPU6050.  We'll pretend that the 
  //angular velocity has remained constant over the time dt, and multiply angular velocity by 
  //time to get displacement.
  //The filter then adds a small correcting factor from the accelerometer ("roll" or "pitch"), so the gyroscope knows which way is down. 
  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch; 
}

float MPU6050cf::angleX( void ) {
  return compAngleX;
}

float MPU6050cf::angleY( void ) {
  return compAngleY;
}

float MPU6050cf::angleZ( void ) {
  return 0; // TODO
}

