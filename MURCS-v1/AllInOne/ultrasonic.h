#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

void initDistanceSensor(void); 
int  getDistance(int debugLevel);

#endif /* ULTRASONIC_H */

