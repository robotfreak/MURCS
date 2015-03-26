#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#include "iomapping.h" 
#include "motorcontrol.h"

const long maxRunTime = 2000; // Maximum run time for motors without additional command. * Should use a command to set this. * 

void initMotorControl()
{
  pinMode(m1SpeedControl,OUTPUT);
  pinMode(m2SpeedControl,OUTPUT);
  pinMode(m1DirectionControl,OUTPUT);
  pinMode(m2DirectionControl,OUTPUT);
}


// Drive DC motors to move the robot using values in range -100 to 100 for left and right
unsigned long driveWheels(int valueLeft, int valueRight) 
{

  // Set left motor pins to turn in the desired direction
  if (valueLeft < 0){
    digitalWrite(m1DirectionControl,HIGH);
  }
  else {
    digitalWrite(m1DirectionControl,LOW);
  }
  // Set right motor pins to turn in the desired direction
  if (valueRight < 0){
    digitalWrite(m2DirectionControl,HIGH);
  }
  else {
    digitalWrite(m2DirectionControl,LOW);
  }
  // Maps "w" values to the wider range that the motor responds to
  valueLeft = map(abs(valueLeft), 0, 100, 0, MAX_SPEED_LIMIT);
  valueRight = map(abs(valueRight), 0, 100, 0, MAX_SPEED_LIMIT);
  analogWrite(m1SpeedControl,valueLeft);
  analogWrite(m2SpeedControl,valueRight);

  return millis() + maxRunTime;
}

