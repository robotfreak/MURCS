#ifndef IOMAPPING_H
#define IOMAPPING_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Ultrasonic pin definition */
#define triggerPin   9
#define echoPin      8  

/* Linesensor pin definition */
#define NUM_LINESENSORS         6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define line1Pin     2
#define line2Pin     3
#define line3Pin     4
#define line4Pin     5
#define line5Pin     6
#define line6Pin     7

#define emitterPin   2  // emitter is controlled by digital pin 2

/* Sharp IR distance sensor definition */
#define wallPin      0
#define frontPin     1

/* Motor Controller Pin definition */
#define m1SpeedControl     13 //M1 Speed Control
#define m2SpeedControl     11 //M2 Speed Control
#define m1DirectionControl 12 //M1 Direction Control
#define m2DirectionControl 10 //M2 Direction Control

/* Wheel encoder pin definition */
#define encoderLtPin   8
#define encoderRtPin   9 

#endif /* IOMAPPING_H */

