#pragma once

#include <Automaton.h>
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

/* Prints string/int/float values to the lcd
     lcd.begin()
     lcd.printXY( 0, 1, "Hello world!" );
     lcd.printXY( 0, 1, intval, 10 );
     lcd.printXY( 0, 1, (uint32_t) 87687687 );
     lcd.setBacklight( lcd.YELLOW );
     lcd.clear();
     lcd.clear( 0, 1 );
*/


class Atm_rgb_lcd: public Machine {

typedef char display_t[2][16];

 public:
  enum { IDLE }; // STATES
  enum { ELSE }; // EVENTS
  enum { RED = 1, GREEN, YELLOW, BLUE, VIOLET, TEAL, WHITE }; // Colors for the backlight

  Atm_rgb_lcd( void ) : Machine() {};
  Atm_rgb_lcd& begin( void );
  Atm_rgb_lcd& trace( Stream & stream );
  Atm_rgb_lcd& trigger( int event );
  int state( void );
  
  // Uses the enum constants defined above: lcd.setBacklight( YELLOW )
  Atm_rgb_lcd& setBacklight( int color );

  // lcd.clear() -> clears the display
  // lcd.clear( 0, 1 ) -> clears the second display line
  // lcd.clear( 5, 1, 12 ) -> clears 12 characters starting at position 5, second line
  Atm_rgb_lcd& clear( int xpos = -1, int ypos = -1, int len = -1 );
  
  // lcd.printXY( 0, 1, "hello" ) -> prints 'hello' at the first position of the second display line 
  // lcd.printXY( 0, 1, "hello", 16 ) -> prints 'hello' in a 16 char wide box ( x=0, y=1) left justified with spaces
  // lcd.printXY( 0, 1, "hello", -16 ) -> prints 'hello' in a 16 char wide box ( x=0, y=1) right justified with spaces
  Atm_rgb_lcd& printXY( int x, int y, const char s[], int justify = 0 );
  
  // Prints a signed or unsigned 8/16/32 integer (justify works as described above)
  Atm_rgb_lcd& printXY( int x, int y, int32_t v, int justify = 0 );
  Atm_rgb_lcd& printXY( int x, int y, uint32_t v, int justify = 0 );
  Atm_rgb_lcd& printXY( int x, int y, int16_t v, int justify = 0 );
  Atm_rgb_lcd& printXY( int x, int y, uint16_t v, int justify = 0 );
  
  // Prints a double/float with the specified precision (justify works as described above)
  Atm_rgb_lcd& printXY( int x, int y, double v, int precision = 2, int justify = 0 );	

  // Simple cursor-based print  
  Atm_rgb_lcd& setCursor( byte cur_x, byte cur_y );
  Atm_rgb_lcd& print( const char s[] );

 private:
  enum { LP_IDLE }; // ACTIONS
  enum { ON_PRESS, CONN_MAX = 5 }; // CONNECTORS
  int event( int id ); 
  void action( int id ); 
  int _updateDisplay( int max_updates );
  byte cur_x, cur_y;
  
  Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

  display_t ist, soll = { // German for 'what is' (ist) and 'what should be' (soll)
    { ' ', ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' },
    { ' ', ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' } };  

};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="Atm_rgb_lcd">
    <states>
      <IDLE index="0">
        <EVT_TIMER>CHECK</EVT_TIMER>
      </IDLE>
      <CHECK index="1">
        <EVT_UPDATE>WRITING</EVT_UPDATE>
      </CHECK>
      <WRITING index="2" on_enter="ENT_WRITING">
        <EVT_UPDATE>WRITING</EVT_UPDATE>
        <ELSE>IDLE</ELSE>
      </WRITING>
    </states>
    <events>
      <EVT_TIMER index="0" access="PRIVATE"/>
      <EVT_UPDATE index="1" access="PRIVATE"/>
    </events>
    <connectors>
      <PRESS autostore="0" broadcast="0" dir="PUSH" slots="5"/>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/

