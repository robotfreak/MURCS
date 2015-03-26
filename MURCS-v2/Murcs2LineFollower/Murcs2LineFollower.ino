/*
 * Demo line-following code for the MURCS v2.0 Robot
 *
 * This code will follow a black line on a white background, using a
 * PID-based algorithm.  It works decently on courses with smooth, 6"
 * radius curves and has been tested with Zumos using 30:1 HP and
 * 75:1 HP motors.  Modifications might be required for it to work
 * well on different courses or with different motors.
 *
 * http://www.pololu.com/catalog/product/2506
 * http://www.pololu.com
 * http://forum.pololu.com
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <QTRSensors.h>

#define NUM_SENSORS   4     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 5, 6},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

int lastError = 0;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *MotorLt = AFMS.getMotor(1);
Adafruit_DCMotor *MotorRt = AFMS.getMotor(2);

// This is the maximum speed the motors will be allowed to turn.
const int MAX_SPEED = 200;

void motors_setSpeeds(int left, int right)
{
  MotorLt->setSpeed(abs(left));
  if (left > 0)
    MotorLt->run(FORWARD);
  else
    MotorLt->run(BACKWARD);
  MotorRt->setSpeed(abs(right));
  if (right > 0)
    MotorRt->run(FORWARD);
  else
    MotorRt->run(BACKWARD);
}

void doLineFollow()
{
  int speed_l, speed_r;
  unsigned int position;

  position = qtrrc.readLine(sensorValues);

  if (position <= 500)
  { // far to the right
    speed_l = MAX_SPEED/2;
    speed_r = -MAX_SPEED/2;
  }  else if (position <= 1000)
  { // far to the right
    speed_l = MAX_SPEED;
    speed_r = 0;
  }
  else if (position <= 1300)
  { // far to the right
    speed_l = MAX_SPEED;
    speed_r = MAX_SPEED/2;
  }
  else if (position <= 1700)
  { // centered on line
    speed_l = MAX_SPEED;
    speed_r = MAX_SPEED;
  }
  else if (position <= 2000)
  { // far to the right
    speed_l = MAX_SPEED/2;
    speed_r = MAX_SPEED;
  }
  else if (position <= 2500)
  { // far to the right
    speed_l = 0;
    speed_r = MAX_SPEED;
  }
  else
  { // far to the left
    speed_l = -MAX_SPEED/2;
    speed_r = MAX_SPEED/2;
  }
  motors_setSpeeds(speed_l, speed_r);
//delay(40);
}

void doLineFollowPID()
{
  // Get the position of the line.  Note that we *must* provide the "sensors"
  // argument to readLine() here, even though we are not interested in the
  // individual sensor readings
  int position = qtrrc.readLine(sensorValues);

  // Our "error" is how far we are away from the center of the line, which
  // corresponds to position 1500.
  int error = position - 1500;

  // Get motor speed difference using proportional and derivative PID terms
  // (the integral term is generally not very useful for line following).
  // Here we are using a proportional constant of 1/4 and a derivative
  // constant of 6, which should work decently for many Zumo motor choices.
  // You probably want to use trial and error to tune these constants for
  // your particular Zumo and line course.
  int speedDifference = error / 4 + 6 * (error - lastError);

  lastError = error;

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int m1Speed = MAX_SPEED + speedDifference;
  int m2Speed = MAX_SPEED - speedDifference;

  // Here we constrain our motor speeds to be between 0 and MAX_SPEED.
  // Generally speaking, one motor will always be turning at MAX_SPEED
  // and the other will be at MAX_SPEED-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you might want to
  // allow the motor speed to go negative so that it can spin in reverse.
  if (m1Speed < 0)
    m1Speed = 0;
  if (m2Speed < 0)
    m2Speed = 0;
  if (m1Speed > MAX_SPEED)
    m1Speed = MAX_SPEED;
  if (m2Speed > MAX_SPEED)
    m2Speed = MAX_SPEED;

  motors_setSpeeds(m1Speed, m2Speed);
}

void setup()
{

  // Turn on LED to indicate we are in calibration mode
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  AFMS.begin();  // create with the default frequency 1.6KHz

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  int i;
  for(i = 0; i < 80; i++)
  {
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
      motors_setSpeeds(-100, 100);
    else
      motors_setSpeeds(100, -100);
    qtrrc.calibrate();

    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(20);
  }
  motors_setSpeeds(0,0);

  // Turn off LED to indicate we are through with calibration
  digitalWrite(13, LOW);
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}

void loop()
{
  doLineFollow();
}
