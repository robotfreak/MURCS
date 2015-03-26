#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define MAX_SPEED_LIMIT   150

void initMotorControl(void); 
unsigned long driveWheels(int valueLeft, int valueRight); 

#endif /* MOTORCONTROL_H */

