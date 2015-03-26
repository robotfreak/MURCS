#ifndef SHARPDISTANCE_H
#define SHARPDISTANCE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

float readGP2D12Range(byte pin, int debugLevel); 
float readGP2D120Range(byte pin, int debugLevel); 

#endif /* SHARPDISTANCE_H */

