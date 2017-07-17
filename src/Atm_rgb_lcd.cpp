#include "Atm_rgb_lcd.hpp"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

Atm_rgb_lcd& Atm_rgb_lcd::begin( ) {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*               ON_ENTER    ON_LOOP  ON_EXIT  ELSE */
    /*    IDLE */          -1,   LP_IDLE,      -1, IDLE,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  lcd.begin( 16, 2 );
  return *this;          
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int Atm_rgb_lcd::event( int id ) {
  switch ( id ) {
  }
  return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 *
 * Available connectors:
 *   push( connectors, ON_PRESS, <sub>, <v>, <up> );
 */

void Atm_rgb_lcd::action( int id ) {
  switch ( id ) {
    case LP_IDLE:
      if ( !_updateDisplay( 1 ) ) sleep();
      return;
  }
}

Atm_rgb_lcd& Atm_rgb_lcd::setBacklight( int color ) {
  lcd.setBacklight( color );
  return *this;
}

Atm_rgb_lcd& Atm_rgb_lcd::setCursor( byte cur_x, byte cur_y ) {
  this->cur_x = constrain( cur_x, 0, 15 );
  this->cur_y = constrain( cur_y, 0, 1  );
  return *this;
}

// Print a string at a specific location optionally clearing space before or after it

Atm_rgb_lcd& Atm_rgb_lcd::printXY( int x, int y, const char s[], int justify /* = 0 */ ) {
  x = constrain( x, 0, 15 );
  y = constrain( y, 0, 1  );
  clear( x, y, abs( justify ) );
  if ( justify < 0 ) x += abs( justify ) - strlen( s );
  for ( uint8_t i = 0; i < strlen( s ); i++ ) {
		if ( x < 16 ) soll[y][x++] = s[i];
  }
  trigger( 0 ); // Wakes up the state machine to update the display
  return *this;
}

// Print a signed 8, 16 or 32 bit integer 

Atm_rgb_lcd& Atm_rgb_lcd::printXY( int x, int y, int32_t v, int justify /* = 0 */ ) {	
  char s[11]; // 10 digits + sign
  itoa( v, s, 10 );
  printXY( x, y, s, justify );
  return *this;
}

// Print an unsigned 8, 16 or 32 bit integer 

Atm_rgb_lcd& Atm_rgb_lcd::printXY( int x, int y, uint32_t v, int justify /* = 0 */ ) {	
  char s[11]; // 10 digits
  ultoa( v, s, 10 );
  printXY( x, y, s, justify );
  return *this;
}

Atm_rgb_lcd& Atm_rgb_lcd::printXY( int x, int y, uint16_t v, int justify /* = 0 */ ) {	
  printXY( x, y, (uint32_t) v, justify );
  return *this;
}

Atm_rgb_lcd& Atm_rgb_lcd::printXY( int x, int y, int16_t v, int justify /* = 0 */ ) {	
  printXY( x, y, (int32_t) v, justify );
  return *this;
}

// Print a float

Atm_rgb_lcd& Atm_rgb_lcd::printXY( int x, int y, double v, int precision, int justify /* = 0 */ ) {	
  char s[32]; // should be enough... 
  dtostrf( v, 0, precision, s );
  printXY( x, y, s, justify );
  return *this;
}

Atm_rgb_lcd& Atm_rgb_lcd::print( const char s[] ) {	
  printXY( cur_x, cur_y, s );
  setCursor( cur_x + strlen( s ), cur_y );
  return *this;
}

Atm_rgb_lcd& Atm_rgb_lcd::clear( int xpos /* = -1 */, int ypos /* = -1 */, int len /* = -1 */ ) {	
  if ( ypos > -1 ) { // Delete from x/y to eol
      int xmax = len >= 0 ? xpos + len : 16;
      for ( uint8_t x = xpos; x < xmax; x++ ) {
        soll[ypos][x] = ' ';
      }    
      setCursor( xpos, ypos );
  } else {
    for ( uint8_t y = 0; y < 2; y++ ) {
      for ( uint8_t x = 0; x < 16; x++ ) {
        soll[y][x] = ' ';
      }
    }
    setCursor( 0, 0 );
  }
  trigger( 0 ); // Wakes up the state machine to update the display
  return *this;
}

int Atm_rgb_lcd::_updateDisplay( int max_updates ) 
{
	uint8_t updates = 0;
	for ( uint8_t y = 0; y < 2; y++ ) {
		for ( uint8_t x = 0; x < 16; x++ ) {
			if ( ist[y][x] != soll[y][x] ) {
				lcd.setCursor( x, y );
				lcd.print( soll[y][x] );
				ist[y][x] = soll[y][x];
				if ( ++updates >= max_updates ) return updates;
			}
		}
	}
	return updates;
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

Atm_rgb_lcd& Atm_rgb_lcd::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int Atm_rgb_lcd::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */


/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

Atm_rgb_lcd& Atm_rgb_lcd::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "RGB_LCD\0ELSE\0IDLE" );
  return *this;
}



