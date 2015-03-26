/* 
 * MiniMURCS - Multi Use Robot Construction System
 *
 * Libs used:  
 * microM - MicroMagician Lib, written by Russell Cameron
 * FastSPI - WS2811 RGB LEDs, written by 
 */

#include <microM.h>
#include <Servo.h>
#include "FastSPI_LED2.h"

#define DEBUG_LEVEL 2

#define WS2811Pin     12
#define frontServoPin 11
#define cliffLtPin    10
#define cliffRtPin     9
//#define triggerPin   10
//#define echoPin       9
#define penServoPin    3

#define encoderLtPin   3
#define encoderRtPin   6
#define maxbotixPin    7

/* global Variables */

// motor control variables
int leftSpeed,rightSpeed = 0;
byte leftBrake, rightBrake = 0;
int speedStep = 10;

// servo control variables
Servo frontServo;
Servo penServo;
int fsPos = 90;
int fsStep = 10;
#define penUpPos 75
#define penDnPos 90

// IR remote control variables
int actIRCommand, lastIRCommand = 0; 

// Ultrasonic sensor variables
long distance;

// Accelerometer variables
int magnitude = 0;

// Encoder  variables
#define ENC_LT 0
#define ENC_RT  1

int encoder[2];
int encRaw[2];
int encMin[2];
int encMax[2];
int encFlag[2];

/* WS2811 Definitions */

#define NUM_LEDS 2 

struct CRGB { 
  byte g; 
  byte r; 
  byte b; 
};

struct CRGB leds[NUM_LEDS];

WS2811Controller800Mhz<12> LED;

/*
void initHCSR04()
{
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(triggerPin, LOW);
}

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  return microseconds / 74 / 2;
}
 
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

long readHCSR04()
{
  long pulsein, distcm;
  // The HC-SR04 is triggered by a HIGH pulse of 10 or more microseconds.
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
 
  // The same pin is used to read the signal from the HC-SR04 a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pulsein = pulseIn(echoPin, HIGH);
  distcm = microsecondsToCentimeters(pulsein);
  if (distcm > 4 && distcm < 200)
  {
    Serial.print("\tDistance:");
    Serial.println(distcm,DEC);        // use the DEC option to convert a byte to a decimal number
  }
  return distcm;
}
*/


int readCliff()
{
  int lt, rt;
  lt = digitalRead(cliffLtPin);
  rt = digitalRead(cliffRtPin);
#if (DEBUG_LEVEL >= 1)
  if (lt || rt)
  {
    Serial.print("\tCliff ");
    if (lt)
    {
      Serial.print("Lt: ");
    }
    if (rt)
    {
      Serial.print(" Rt: ");
    }
    Serial.println("");
  }
#endif  
}

int readMaxbotix()
{
  int dist;
  dist = analogRead(maxbotixPin)*1000/548;
#if (DEBUG_LEVEL >= 1)
  Serial.print("\tDistance: ");
  Serial.println(dist, DEC);
#endif  
  return dist;
}

int readIR(void)
{
  int irCmd = 0;
  if(microM.ircommand>0)                         // display command from IR receiver
  {
    irCmd = microM.ircommand;
#if (DEBUG_LEVEL >= 1)
    Serial.print("\tIR command:");
    Serial.println(microM.ircommand,DEC);        // use the DEC option to convert a byte to a decimal number
#endif
    microM.ircommand=0;                          // prevents display repeating the same values
  }
  return irCmd; 
}

void readAccel()
{
  microM.Impact();                       // must be included in your loop for impact detection to work
  if(microM.magnitude>0)                 // display results of impact
  {
    magnitude = microM.magnitude;
#if (DEBUG_LEVEL >= 1)
    Serial.println(""); 
    Serial.print("\tMagnitude:");
    Serial.print(microM.magnitude);
    Serial.print("\tDelta X:");
    Serial.print(microM.deltx);
    Serial.print("\tDelta Y:");
    Serial.print(microM.delty);
    Serial.print("\tDelta Z:");
    Serial.println(microM.deltz);
    Serial.println(""); 
#endif
    microM.magnitude=0;                  // prevents display repeating the same values
  }
}

void initEncoder()
{
  for(int i=0; i<2; i++)
  {
    encoder[i] = 0;
    encMin[i] = 10000;
    encMax[i] = 0;
    encFlag[i] = 0;
  }
}

void readEncoder()
{
 
  encRaw[ENC_LT] = analogRead(encoderLtPin);
  encRaw[ENC_RT] = analogRead(encoderRtPin);
  for(int i=0; i<2; i++)
  {
    if (encRaw[i] < 400 && encFlag[i] == 1)
    {
      encoder[i]++;
      encFlag[i] = 0;
#if (DEBUG_LEVEL >= 1)
      Serial.print("Enc: ");
      Serial.print(i, DEC);
      Serial.print(" ");
      Serial.println(encoder[i], DEC);
#endif
    }
    if (encRaw[i] > 450 && encFlag[i] == 0)
    {
      encoder[i]++;
      encFlag[i] = 1;
#if (DEBUG_LEVEL >= 1)
      Serial.print("Enc: ");
      Serial.print(i, DEC);
      Serial.print(" ");
      Serial.println(encoder[i], DEC);
#endif
    }
    if (encRaw[i] < encMin[i]) 
    {
      encMin[i] = encRaw[i];
#if (DEBUG_LEVEL >= 2)
      Serial.print("Min: ");
      Serial.print(i, DEC);
      Serial.print(" ");
      Serial.println(encMin[i], DEC);
#endif
    }
    if (encRaw[i] > encMax[i]) 
    {
      encMax[i] = encRaw[i];
#if (DEBUG_LEVEL >= 2)
      Serial.print("Max: ");
      Serial.print(i, DEC);
      Serial.print(" ");
      Serial.println(encMax[i], DEC);
#endif
    }
  }
}

void rampMotors()
{
  if (leftSpeed >= 250)
    speedStep = -10;
  if (leftSpeed <= -250)
    speedStep = 10;
  if (leftSpeed >= -250 && leftSpeed <=250)
    leftSpeed += speedStep;
    
  if (rightSpeed >=250)
    speedStep = -10;
  if (rightSpeed <= -250)
    speedStep = 10;
  if (rightSpeed >= -250 && rightSpeed <=250)
    rightSpeed += speedStep;
#if (DEBUG_LEVEL >= 1)
  Serial.print("\tSpeed Lt: ");   
  Serial.print(leftSpeed, DEC);
  Serial.print(" Rt: ");   
  Serial.println(rightSpeed, DEC);
#endif
}

void writeLEDs(byte r, byte g, byte b)
{
  for(int iLed = 0; iLed < NUM_LEDS; iLed++) 
  {
    leds[iLed].r = r; 
    leds[iLed].r = g; 
    leds[iLed].r = b; 
  }
}
void readSensors()
{
  microM.Impact();                       // must be included in your loop for impact detection to work
  readCliff();
//  distance = readHCSR04();
//  distance = readMaxbotix();
  readEncoder();
  readAccel();                       // must be included in your loop for impact detection to work
  
  actIRCommand = readIR();
  if (actIRCommand && actIRCommand != lastIRCommand)
  {
    // todo: update Mode of operation
    switch (actIRCommand)
    {
      case 53: // >>
        if (fsPos < 160)
          fsPos += fsStep;
      break;
      case 52: // <<  
        if (fsPos > 20)
          fsPos -= fsStep;
      break;
      case 49: // |<<
        penServo.write(penDnPos);
      break;
      case 50: // >>|
        penServo.write(penUpPos);
      break;
      case 57:  //stop
      case 5:
        leftSpeed = 0;
        rightSpeed = 0;
      break;
      case 2:
        leftSpeed = 150;
        rightSpeed = 150;
      break;
      case 8:
        leftSpeed = -150;
        rightSpeed = -150;
      break;
      case 4:
        leftSpeed = -150;
        rightSpeed = 150;
      break;
      case 6:
        leftSpeed = 150;
        rightSpeed = -150;
      break;
      case 1:
        leftSpeed = 0;
        rightSpeed = 150;
      break;
      case 3:
        leftSpeed = 150;
        rightSpeed = 0;
      break;
      case 7:
        leftSpeed = -150;
        rightSpeed = 0;
      break;
      case 9:
        leftSpeed = 0;
        rightSpeed = -150;
      break;
      
    }
    lastIRCommand = actIRCommand ;
  }
  else
    lastIRCommand = 0;
}

void updateActors()
{
  microM.Motors(leftSpeed,rightSpeed,leftBrake,rightBrake);    // update motor controller
  microM.Impact();                       // must be included in your loop for impact detection to work
   frontServo.write(fsPos);
  microM.Impact();                       // must be included in your loop for impact detection to work
//  LED.showRGB((byte*)leds, NUM_LEDS);
}

void doProcessing()
{
//  rampMotors();
}

void setup() 
{
  Serial.begin(38400);
  Serial.println("MiniMURCS v0.1");
  Serial.println("resetting!");

  microM.Setup(); 
  microM.sensitivity=30;                 // if your robot vibrates or moves over a rough surface then increase this value to prevent false triggering
  microM.devibrate=100;                  // depends on the construction of your robot. Increase the value if you get additional readings after initial impact

  initEncoder();
  LED.init();
  memset(leds, 0,  NUM_LEDS * sizeof(struct CRGB));    
  writeLEDs(0,128,0); // Set LEDs to green
  LED.showRGB((byte*)leds, NUM_LEDS);

  frontServo.attach(frontServoPin);
  frontServo.write(90);

  penServo.attach(penServoPin);
  penServo.write(penDnPos);
  delay(2000);
  penServo.write(penUpPos);

//  initHCSR04();

  delay(2000);
}


void loop()
{
  readSensors();
  doProcessing();
  updateActors();
}

