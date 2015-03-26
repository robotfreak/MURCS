/*
	ObstacleAvoiding.ino - MURCS 
 	Author: RobotFreak www.robotfreak.de
 	Copyright (c) 2013 RobotFreak All Rights Reserved
 
 	This library is free software; you can redistribute it and/or
 	modify it under the terms of the GNU Lesser General Public
 	License as published by the Free Software Foundation; either
 	version 2.1 of the License, or (at your option) any later version.
 
 	This library is distributed in the hope that it will be useful,
 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 	Lesser General Public License for more details.
 
 	You should have received a copy of the GNU Lesser General Public
 	License along with this library; if not, write to the Free Software
 	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 */
#include <inttypes.h> 

#define DEBUG_LEVEL 2

/* HC-SR04 pin definition */
#define triggerPin   9
#define echoPin      8  

/* Motor driver pin definition */
#define Pwm1Pin     13  //M1 Speed Control
#define Pwm2Pin     11  //M2 Speed Control
#define Dir1Pin     12  //M1 Direction Control
#define Dir2Pin     10  //M1 Direction Control

#define MAX_SPEED  100  // speed_limit

static int last_speed1, last_speed2;
int speed_l;
int speed_r;
int dly;
int distance;

void motorInit(void)
{
  pinMode(Dir1Pin, OUTPUT);
  pinMode(Dir2Pin, OUTPUT);
}

void motorMove(int speed1, int speed2)
{

  last_speed1 = speed1;
  last_speed2 = speed2;

  if (speed1 < 0)
  {
    digitalWrite(Dir1Pin, HIGH);
    analogWrite(Pwm1Pin, abs(speed1));
  }
  else
  {
    digitalWrite(Dir1Pin, LOW);
    analogWrite(Pwm1Pin, abs(speed1));
  }
  
  if (speed2 < 0)
  {
    digitalWrite(Dir2Pin, HIGH);
    analogWrite(Pwm2Pin, abs(speed2));
  }
  else
  {
    digitalWrite(Dir2Pin, LOW);
    analogWrite(Pwm2Pin, abs(speed2));
  }
  
}

void motorMoveRamp(int speed1, int speed2, int dly)
{
  int temp_speed1 = last_speed1;
  int temp_speed2 = last_speed2;

  Serial.print("MoveRamp: ");
  Serial.print(speed1, DEC);
  Serial.print(", ");
  Serial.print(speed2, DEC);
  Serial.print(", ");
  Serial.println(dly, DEC);
  while ((speed1 != temp_speed1) ||
         (speed2 != temp_speed2))
   {
      if (temp_speed1 > speed1)
        temp_speed1--;
      else if (temp_speed1 < speed1)
        temp_speed1++;

      if (temp_speed2 > speed2)
        temp_speed2--;
      else if (temp_speed2 < speed2)
        temp_speed2++;
      motorMove(temp_speed1, temp_speed2);
      delay(dly);
   }
}

void setup() {

  Serial.begin(38400);                 // Sets the baud rate to 38400
  motorInit();
  initHCSR04();
  speed_l = 0;
  speed_r = 0;
  Serial.println("MURCS obstacle Avoiding Version 1.0");   
}

  
void loop()
{
  int i;
  
  distance = getDistance();  // get distance
  Serial.print("Distance: ");
  Serial.println(distance, DEC);
/*  
  Serial.println("Forward increase: ");
  for(i= 0; i <= MAX_SPEED; i+= 10)
  {
    motorMove(i,i);
    delay(200);
  }
  Serial.println("Forward decrease: ");
  for(i= MAX_SPEED; i >= 0 ; i-= 10)
  {
    motorMove(i,i);
    delay(200);
  }
  delay(2000);
  Serial.println("Backward increase: ");
  for(i= 0; i >= -MAX_SPEED ; i-= 10)
  {
    motorMove(i,i);
    delay(200);
  }
  Serial.println("Backward decrease: ");
  for(i= -MAX_SPEED; i <= 0 ; i+= 10)
  {
    motorMove(i,i);
    delay(200);
  }
  delay(2000);
*/

  if (distance > 0)
  {
  if (distance <= 10)
  {
    motorMove(0,0);
    delay(500);
    speed_l = -50;
    speed_r = 50;
    dly = 5;
    do {
      distance = getDistance();  // get distance
      delay(40);
//      motorMove(speed_l, speed_r);
      motorMoveRamp(speed_l, speed_r, dly);
    } while(distance < 20);  
    speed_l = 0;
    speed_r = 0;
//    motorMove(speed_l,speed_r);
    motorMoveRamp(speed_l, speed_r, dly);
    delay(500);
  }
  else if (distance < 30)
  {
    speed_l = 70;
    speed_r = 70;
  }
  else
  {
    speed_l = MAX_SPEED;
    speed_r = MAX_SPEED;
  }
  dly = 10;
//  motorMove(speed_l, speed_r);
  motorMoveRamp(speed_l, speed_r, dly);
  }
  
  delay(40);

}

void initHCSR04()
{
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(triggerPin, LOW);
}

int microsecondsToInches(int microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  return microseconds / 74 / 2;
}
 
int microsecondsToCentimeters(int microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

int getDistance()
{
  int pulsein, distcm;
  // The HC-SR04 is triggered by a HIGH pulse of 10 or more microseconds.
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
 
  // The same pin is used to read the signal from the HC-SR04 a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pulsein = pulseIn(echoPin, HIGH);
  distcm = microsecondsToCentimeters(pulsein);
#if (DEBUG_LEVEL >= 1) 
  if (distcm > 4 && distcm < 200)
  {
    Serial.print("\tDistance:");
    Serial.println(distcm,DEC);        // use the DEC option to convert a byte to a decimal number
  }
#endif
  return distcm;
} 








