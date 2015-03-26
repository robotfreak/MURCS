#ifndef ENCODER_H
#define ENCODER_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define ENC_LT 0
#define ENC_RT 1

 
void stopEncoder(void); 
void startEncoder(void); 
void setEncoder(int lt, int rt);
void initEncoder(void);
void readEncoder(int debugLevel);

#endif /* ENCODER_H */

