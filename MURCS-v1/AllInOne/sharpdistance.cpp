#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*
 * sharpdistance.cpp Sharp GP2Dxx IR sensor functions
 * taken from https://code.google.com/p/rbots/
 *
 */

float readGP2D12Range(byte pin, int debugLevel) 
{
  float val;
  int tmp;
  
  tmp = analogRead(pin);
  if(tmp < 3)
    return -1;  //invalid value
    
  val = (6787.0 / ((float)tmp - 3.0))-4.0;
  if (debugLevel > 1) 
  {
    Serial.print("Distance: ");
    Serial.print(val, DEC);
    Serial.println(" cm");
  }
  return val;
}

float readGP2D120Range(byte pin, int debugLevel) 
{
  float val;
  int tmp;
  
  tmp = analogRead(pin);
  if(tmp < 3)
    return -1;  //invalid value
    
  val = (2914.0 / ((float)tmp + 5.0))-1.0;
  if (debugLevel > 1) 
  {
    Serial.print("Distance: ");
    Serial.print(val, DEC);
    Serial.println(" cm");
  }
  return val;
}

