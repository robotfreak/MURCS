/*
 AllInOne.ino - MURCS robot commanded by serial input
 
 Looks for a set of ASCII characters in the signal to send
 commands to drive a small robot. 
 
 The minimum circuit:
 * Serial input connected to RX pin 0
 * Serial output connected to TX pin 1

 * optional Bluetooth Module connected to Serial2
 
 Note: If you don't yet have a serial device to connect with, you can use the 
 built in Serial Monitor in the Arduino software when connect via USB for testing.
 Also, be sure to disconect RX & TX pins from other devices when trying to program
 the Arduino over USB.
 
 based on cellbots.pde, created 2010
 by Tim Heath, Ryan Hickman, and Glen Arrowsmith
 Visit http://www.cellbots.com for more information
 
 adapted for MURCS robot by RobotFreak 2013
 Visit http://www.robotfreak.de for more information
*/
#include <Servo.h>
#include <QTRSensors.h>
#include <PID_v1.h>
#include "iomapping.h"
#include "motorcontrol.h"
#include "encoder.h"
//#include "ultrasonic.h"
#include "sharpdistance.h"

#define BUFFERSIZE 20
#define DEFAULT_speedMultiplier 5 
#define USE_BLUETOOTH_SERIAL2    // we use Bluetooth over Serial2

#define MODE_REMOTE_CONTROL  1
#define MODE_LINE_FOLLOW     2
#define MODE_LINE_FOLLOW_PID 3
#define MODE_WALL_FOLLOW     4
#define MODE_OBSTACLE_AVOID  5

// ** GENERAL SETTINGS ** - General preference settings
#define DEBUG_LEVEL 2
int debugLevel = DEBUG_LEVEL;
boolean DEBUGGING = true; // Whether debugging output over serial is on by defauly (can be flipped with 'h' command)

int operationMode = MODE_REMOTE_CONTROL;


int speedMultiplier = DEFAULT_speedMultiplier; // Default speed setting. Uses a range from 1-10 

// No config required for these parameters
unsigned long stopTime=millis(); // used for calculating the run time for motors
char incomingByte; // Holds incoming serial values
char msg[8]; // For passing back serial messages
char inBytes[BUFFERSIZE]; //Buffer for serial in messages
int serialIndex = 0; 
int serialAvail = 0;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.0, consKd=0.0;
//double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

Servo myServo;

void setup() 
{
  initMotorControl();
//  initEncoder();
  myServo.attach(servoPin);
  myServo.write(90);

  Serial.begin(9600);
#ifdef USE_BLUETOOTH_SERIAL2
  Serial2.begin(115200);
#endif
  if (debugLevel) { Serial.println("MURCS AllInOne V1.0");  }
//  initLineFollowPID();
  
} 


// Stop the bot
void stopBot() 
{
  driveWheels(0,0);
  if (debugLevel){ Serial.println("Stopping both wheels"); 
  }
  serialReply("i", "st"); // Tell the phone that the robot stopped
}

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {line1Pin, line2Pin, line3Pin, line4Pin, line5Pin, line6Pin}, 
  NUM_LINESENSORS, NUM_SAMPLES_PER_SENSOR, emitterPin);
unsigned int sensorValues[NUM_LINESENSORS];


void initLineSensors(int debugLevel)
{
  delay(1000);
  for (int i = 0; i < 30; i++)  // make the calibration take about 1.25 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    driveWheels(-20,20);
  }
  driveWheels(0,0);
  delay(500);
  for (int i = 0; i < 60; i++)  // make the calibration take about 2.5 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    driveWheels(20,-20);
  }
  driveWheels(0,0);
  delay(500);
  for (int i = 0; i < 30; i++)  // make the calibration take about 1.25 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    driveWheels(-20,20);
  }
  driveWheels(0,0);
  delay(500);
  // print the calibration minimum values measured when emitters were on
  if (debugLevel > 1)
  {
    for (int i = 0; i < NUM_LINESENSORS; i++)
    {
      Serial.print(qtra.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
  
    // print the calibration maximum values measured when emitters were on
    for (int i = 0; i < NUM_LINESENSORS; i++)
    {
      Serial.print(qtra.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  }
  delay(1000);
}


unsigned int  readLineSensors(int debugLevel)
{
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtra.read(sensorValues); instead of unsigned int position = qtra.readLine(sensorValues);
  unsigned int position = qtra.readLine(sensorValues);
  static int count = 0;
  
  if (debugLevel > 1)
  {
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
    for (unsigned char i = 0; i < NUM_LINESENSORS; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    //Serial.println(); // uncomment this line if you are using raw values
    Serial.println(position); // comment this line out if you are using raw values
  }
  if (count > 10)
  {
    Serial2.write((position >> 8) & 0xFF);
    Serial2.write(position & 0xFF);
    count = 0;
  }
  return position;
}

void initRemoteControl()
{
  myServo.write(90);
}

void initLineFollow()
{
  myServo.write(90);
  initLineSensors(debugLevel);
}

void initLineFollowPID()
{
  myServo.write(90);
  initLineSensors(debugLevel);
  Input = 0.0;
  Setpoint = 0.0;
  //tell the PID to range between -50 and 50 
  myPID.SetOutputLimits(-2000, 2000); 
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void initWallFollow()
{
  myServo.write(180);
}

void initObstacleAvoid()
{
  myServo.write(90);
//  initDistanceSensor();
}

void changeMode(int newMode)
{
  if (newMode != operationMode)
  {
    stopBot();
    operationMode = newMode;
    switch(operationMode)
    {
      case MODE_REMOTE_CONTROL: 
        if (debugLevel) { Serial.println("Remote Control");  }
        initRemoteControl();
      break;
      case MODE_LINE_FOLLOW: 
        if (debugLevel) { Serial.println("Line Follow");  }
        initLineFollow();
      break;
      case MODE_LINE_FOLLOW_PID: 
        if (debugLevel) { Serial.println("Line Follow PID");  }
        initLineFollowPID();
      break;
      case MODE_WALL_FOLLOW: 
        if (DEBUGGING) { Serial.println("Wall Follow");  }
        initWallFollow();
      break;
      case MODE_OBSTACLE_AVOID: 
        if (debugLevel) { Serial.println("Obstacle Avoiding");  }
        initObstacleAvoid();
      break;
    }
  }
}

// Replies out over serial and handles pausing and flushing the data to deal with Android serial comms
void serialReply(char* sensorname, char* tmpmsg) 
{
#ifdef USE_BLUETOOTH_SERIAL2
  Serial2.print(sensorname);
  Serial2.print(":");
  Serial2.println(tmpmsg); // Send the message back out the serial line
  //Wait for the serial debugger to shut up
  delay(200); //this is a magic number
  Serial2.flush(); //clears all incoming data
#else
  Serial.print(sensorname);
  Serial.print(":");
  Serial.println(tmpmsg); // Send the message back out the serial line
  //Wait for the serial debugger to shut up
  delay(200); //this is a magic number
  Serial.flush(); //clears all incoming data
#endif
}

// Check if enough time has elapsed to stop the bot and if it is safe to proceed
void checkIfStopBot() 
{
  if (stopTime < millis()) 
  {
    stopBot();
  }
}


// Send command to attached Bluetooth device to initiate pairing
void pairBluetooth() 
{
#ifdef USE_BLUETOOTH_SERIAL2
//  Serial2.print("\r\n+INQ=1\r\n"); // This is for Seeedstudio master/slave unit (change as needed for your model)
#else
  Serial.print("\r\n+INQ=1\r\n"); // This is for Seeedstudio master/slave unit (change as needed for your model)
#endif
}

// Reads serial input if available and parses command when full command has been sent. 
void readSerialInput() 
{
#ifdef USE_BLUETOOTH_SERIAL2
  while(Serial2.available() && serialIndex < BUFFERSIZE) {
#else
  while(Serial.available() && serialIndex < BUFFERSIZE) {
#endif
    //Store into buffer.
    inBytes[serialIndex] = Serial2.read();

    //Check for command end.    
    if (inBytes[serialIndex] == '\n' || inBytes[serialIndex] == ';' || inBytes[serialIndex] == '>') 
    { //Use ; when using Serial Monitor
       inBytes[serialIndex] = '\0'; //end of string char
       parseCommand(inBytes); 
       serialIndex = 0;
    }
    else
    {
      serialIndex++;
    }
  }
  
  if(serialIndex >= BUFFERSIZE)
  {
    //buffer overflow, reset the buffer and do nothing
    //TODO: perhaps some sort of feedback to the user?
    for(int j=0; j < BUFFERSIZE; j++)
    {
      inBytes[j] = 0;
      serialIndex = 0;
    }
  }
}

// Cleans and parses the command
void parseCommand(char* com) 
{
  if (com[0] == '\0') { return; } //bit of error checking
  int start = 0;
  //get start of command
  while (com[start] != '<')
  {
    start++; 
    if (com[start] == '\0') 
    {
      //its not there. Must be old version
      start = -1;
      break;
    }
  }
  start++;
  performCommand(com);
}

void performCommand(char* com) 
{  
  if (strcmp(com, "f") == 0) 
  { // Forward
    stopTime = driveWheels(speedMultiplier * 10, speedMultiplier * 10);
  } else if (strcmp(com, "r") == 0) 
  { // Right
    stopTime = driveWheels(speedMultiplier * 10, speedMultiplier * -10);
  } else if (strcmp(com, "l") == 0) 
  { // Left
    stopTime = driveWheels(speedMultiplier * -10, speedMultiplier * 10);
  } else if (strcmp(com, "b") == 0) 
  { // Backward
    stopTime = driveWheels(speedMultiplier * -10, speedMultiplier * -10);
  } else if (strcmp(com, "s") == 0) 
  { // Stop
    stopBot();
  } else if (strcmp(com, "R") == 0) 
  { // Set Mode Remote Control
    changeMode(MODE_REMOTE_CONTROL);
  } else if (strcmp(com, "L") == 0) 
  { // Set Mode Line Follower
    changeMode(MODE_LINE_FOLLOW);
  } else if (strcmp(com, "P") == 0) 
  { // Set Mode Line Follower PID
    changeMode(MODE_LINE_FOLLOW_PID);
  } else if (strcmp(com, "W") == 0) 
  { // Set Mode Wall Follower
    changeMode(MODE_WALL_FOLLOW);
  } else if (strcmp(com, "O") == 0) 
  { // Set Mode Obstacle Avoider
    changeMode(MODE_OBSTACLE_AVOID);
  } else if (strcmp(com, "h") == 0) 
  { // Help mode - debugging toggle
    // Print out some basic instructions when first turning on debugging
    if (debugLevel == 0) {
      Serial.println("Ready to listen to commands! Try ome of these:");
      Serial.println("F (forward), B (backward), L (left), R (right), S (stop), D (demo).");
      Serial.println("Also use numbers 1-9 to adjust speed (0=slow, 9=fast).");
    }
    debugLevel = !debugLevel;
  } else if (strcmp(com, "1") == 0 || strcmp(com, "2") == 0 || strcmp(com, "3") == 0 || strcmp(com, "4") == 0 || strcmp(com, "5") == 0 || strcmp(com, "6") == 0 || strcmp(com, "7") == 0 || strcmp(com, "8") == 0 || strcmp(com, "9") == 0 || strcmp(com, "0") == 0) 
  {
    //I know the preceeding condition is dodgy but it will change soon 
    if (debugLevel) { Serial.print("Changing speed to "); }
    int i = com[0];
    speedMultiplier = i - 48; // Set the speed multiplier to a range 1-10 from ASCII inputs 0-9
    if (debugLevel) { Serial.println(speedMultiplier); }
  } else if (com[0] == 'w') 
  { // Handle "wheel" command and translate into PWM values ex: "w -100 100" [range is from -100 to 100]
    int valueLeft=90, valueRight=90;
    sscanf (com,"w %d %d",&valueLeft, &valueRight); // Parse the input into multiple values
    stopTime = driveWheels(valueLeft, valueRight);
  } else if (com[0] == 'p') 
  { // Initiates Bluetooth pairing so another device can connect
    pairBluetooth();
  } else 
  { 
    serialReply("e", com);// Echo unknown command back
    if (debugLevel) 
    {
      Serial.print("Unknown command: ");
      Serial.println(com);
    }
  }
}

void doLineFollow()
{
  int speed_l, speed_r;
  unsigned int position;

  position = readLineSensors(debugLevel);

  if (position <= 1000)
  {  // far to the right
    if (debugLevel > 1) { Serial.println("far to the right");}
    speed_l = 40;
    speed_r = 0;
  }
  else if (position <= 3000)
  {  // centered on line 
    if (debugLevel > 1) { Serial.println("on line");}
    speed_l = 40;
    speed_r = 40;
  }
  else
  {  // far to the left
    if (debugLevel > 1) { Serial.println("far to the left");}
    speed_l = 0;
    speed_r = 40;
  }
  stopTime = driveWheels(speed_l, speed_r);
  delay(40); 
}

void doLineFollowPID()
{
  int speed_l, speed_r, speedDiff;
  int position;

  position = readLineSensors(0);
  Input = position - 2000.0;
  myPID.Compute();

  speedDiff = map(abs((int)Output), -2000, 2000, -50, 50);
//  if (Output < 0.0)
//  speedDiff = -speedDiff;
  if (debugLevel > 1) 
  {
    Serial.print("Input: ");
    Serial.print(Input, DEC);
    Serial.print(" Output: ");
    Serial.print(Output, DEC);
    Serial.print(" SDiff: ");
    Serial.println(speedDiff, DEC);
  }
//  speedDiff = (int) power_difference;
  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  const int max = 50;
  if(speedDiff > max)
      speedDiff = max;
  if(speedDiff < -max)
      speedDiff = -max;
 
  if(speedDiff < 0)
     stopTime = driveWheels(max+speedDiff, max);
  else
     stopTime =  driveWheels(max, max-speedDiff);
  delay(40); 
}

void doWallFollow()
{
  int speed_l, speed_r;
  float distance;
  float wallDistance;

  distance = readGP2D120Range(frontPin, debugLevel);
  if (distance > 0.0)
  {
    if (distance <= 10.0)
    {
      if (debugLevel > 1) { Serial.println("Obstacle ahead");}
      driveWheels(0,0);
      delay(500);
      speed_l = -40;
      speed_r = 40;
      do {
          distance = readGP2D120Range(frontPin, debugLevel);
        delay(40);
        driveWheels(speed_l, speed_r);
      } while(distance < 20.0);  
      if (debugLevel > 1) { Serial.println("turn away from Obstacle");}
      speed_l = 0;
      speed_r = 0;
      driveWheels(speed_l,speed_r);
      delay(500);
    }
    else
    {
      // IR sensor returns distance in centimeters
      wallDistance = readGP2D120Range(wallPin, debugLevel);
      if (wallDistance > 0.0)
      {
        if (wallDistance < 8.0)
        { // if we are way too close, turn away fast
            if (debugLevel > 1) { Serial.println("too close to wall");}
            speed_l = 0;
            speed_r = 50;
        } else if (wallDistance < 12.0)
        { // if we are too close, drift away
            if (debugLevel > 1) { Serial.println("move away from wall");}
            speed_l = 30;
            speed_r = 50;
        } else if (wallDistance < 16.0)
        { // default, drift towards wall
            if (debugLevel > 1) { Serial.println("move closer to wall");}
            speed_l = 50;
            speed_r = 30;
        } else
        { // too far away, turn towards wall more
            if (debugLevel > 1) { Serial.println("too far away from wall");}
            speed_l = 50;
            speed_r = 20;
        }
        stopTime = driveWheels(speed_l, speed_r);
      }
    }
  }
  delay(40); 
}

void doObstacleAvoid()
{
  int speed_l, speed_r;
  float distance;

//  distance = getDistance(debugLevel);
  distance = readGP2D120Range(frontPin, debugLevel);
  if (distance > 0.0)
  {
    if (distance <= 10.0)
    {
      if (debugLevel > 1) { Serial.println("Obstacle ahead");}
      driveWheels(0,0);
      delay(500);
      speed_l = -40;
      speed_r = 40;
      do {
//        distance = getDistance(debugLevel);  // get distance
          distance = readGP2D120Range(frontPin, debugLevel);
        delay(40);
        driveWheels(speed_l, speed_r);
      } while(distance < 20.0);  
      if (debugLevel > 1) { Serial.println("turn away from Obstacle");}
      speed_l = 0;
      speed_r = 0;
      driveWheels(speed_l,speed_r);
      delay(500);
    }
    else
    {
      speed_l = 50;
      speed_r = 50;
    }
    stopTime = driveWheels(speed_l, speed_r);
  }
  
  delay(40); 

}

// Main loop running at all times
void loop() 
{
  readSerialInput();
  switch(operationMode)
  {
    case MODE_LINE_FOLLOW: 
      doLineFollow();
    break;
    case MODE_LINE_FOLLOW_PID: 
      doLineFollowPID();
    break;
    case MODE_WALL_FOLLOW: 
      doWallFollow();
    break;
    case MODE_OBSTACLE_AVOID: 
      doObstacleAvoid();
    break;
  }
  checkIfStopBot();
}

