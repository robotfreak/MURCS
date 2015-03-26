/* 
 * MiniMURCS - Multi Use Robot Construction System (by RobotFreak)
 *
 * Libs used:  
 * microM - MicroMagician Lib, written by Russell Cameron
 * FastSPI - WS2811 RGB LEDs, written by 
 * Go, Turn, Encoder functions taken from Asuro Lib, written by Jan Grewe (DLR), Robotrixer, Stochri, Waste, Andun, m.a.r.v.i.n (Roboternetz.de)
 */

#include <microM.h>
#include <Servo.h>

#define DEBUG_LEVEL 2

#define  FALSE	0
#define  TRUE	1

#define DEG45   10
#define DEG90   20
#define DEG135  31
#define DEG180  40

/* global Variables */

// motor control variables
int leftSpeed,rightSpeed = 0;
byte leftBrake, rightBrake = 0;
int speedStep = 10;

// servo control variables
#define penServoPin    3


#define PEN_UP   0
#define PEN_DOWN 1
#define penUpPos 75
#define penDnPos 90

Servo penServo;
int penPos;

// IR remote control variables
int actIRCommand, lastIRCommand = 0; 

// Encoder  variables
#define encoderLtPin   3
#define encoderRtPin   6

#define ENC_LT 0
#define ENC_RT 1

volatile int encoder[2];
volatile int encRaw[2];
int encMin[2];
int encMax[2];
int encFlag[2];
volatile int autoencode=FALSE;

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

void stopEncoder(void) 
{ 
  autoencode=FALSE; 
}

void startEncoder(void) 
{ 
  autoencode=TRUE; 
}

void setEncoder(int lt, int rt)
{
  encoder[0] = lt;
  encoder[1] = rt;
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
  cli();
 //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  
  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
  ADMUX |= encoderLtPin;
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
//  ADCSRA = (1<<ADEN) | (1<<ADFR) | (1<<ADIE) | (1<<ADSC) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2); // clk/128
//  ADMUX = (1<<ADLAR) | (1<<REFS0) | encoderLtPin; // AVCC reference with external capacitor
  autoencode=TRUE;
  sei();
  setEncoder(0,0);
}

void readEncoder()
{
  if (autoencode)
  {
#if (DEBUG_LEVEL >= 1)
    Serial.print("Enc: ");
    Serial.print(" ");
    Serial.print(encoder[ENC_LT], DEC);
    Serial.print(" ");
    Serial.println(encoder[ENC_RT], DEC);
//    Serial.print("Raw: ");
//    Serial.print(" ");
//    Serial.print(encRaw[ENC_LT], DEC);
//    Serial.print(" ");
//    Serial.println(encRaw[ENC_RT], DEC);
#endif
  }
  else
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

}

void Pen(int pos)
{
  if (pos == PEN_UP)
    penPos = penUpPos;
  else
    penPos = penDnPos;
  penServo.write(penPos);
  delay(300);
}

void Go(int distance, int speed)
{
  long enc_count = 0;
  long tot_count = 0;
  int diff = 0;
  int l_speed = speed, r_speed = speed;

  enc_count=abs(distance);
#if (DEBUG_LEVEL >= 1)
  Serial.print("Go: ");
  Serial.print(distance, DEC);
  Serial.print(" ");
  Serial.println(speed, DEC);
#endif

//  enc_count=distance*10000L;
//  enc_count/=12823L;
  
  setEncoder(0,0);		// reset encoder

  microM.Motors(l_speed,r_speed, 0, 0);

  while(tot_count<enc_count) 
  {
    tot_count += encoder[ENC_LT];
    diff = encoder[ENC_LT] - encoder[ENC_RT];
    if (diff > 0) { //Left faster than right
      if ((l_speed > speed) || (r_speed > 244)) l_speed -= 10;
      else r_speed += 10;
    }
    if (diff < 0) { //Right faster than left
      if ((r_speed > speed) || (l_speed > 244)) r_speed -= 10;
      else l_speed += 10;
    }
    setEncoder(0,0);		// reset encoder
    microM.Motors(l_speed,r_speed, 0, 0);
    delay(10);
  }
  microM.Motors(0,0, 1,1);
  delay(200);
}

void Turn(int degree, int speed)
{
  long enc_count;
  long tot_count = 0;
  int diff = 0;
  int l_speed = speed, r_speed = speed;

   enc_count = abs(degree);
//  enc_count=abs(degree)*0166L; 
//  enc_count /= 0360L;


#if (DEBUG_LEVEL >= 1)
  Serial.print("Turn: ");
  Serial.print(degree, DEC);
  Serial.print(" ");
  Serial.println(speed, DEC);
#endif


  setEncoder(0,0);		// reset encoder

  if (degree < 0)
    microM.Motors(-l_speed,r_speed, 0, 0);
  else
    microM.Motors(l_speed,-r_speed, 0, 0);
  while(tot_count<enc_count) 
  {
    tot_count += encoder[ENC_LT];
    diff = encoder[ENC_LT] - encoder[ENC_RT];
    if (diff > 0) { //Left faster than right
      if ((l_speed > speed) || (r_speed > 244)) l_speed -= 10;
      else r_speed += 10;
    }
    if (diff < 0) { //Right faster than left
      if ((r_speed > speed) || (l_speed > 244)) r_speed -= 10;
      else l_speed += 10;
    }
    setEncoder(0,0);		// reset encoder
    if (degree < 0)
      microM.Motors(-l_speed,r_speed, 0, 0);
    else
      microM.Motors(l_speed,-r_speed, 0, 0);
    delay(10);
  }
  microM.Motors(0,0, 1,1);
  delay(200);
}


void readSensors()
{
//  readEncoder();

  actIRCommand = readIR();
  if (actIRCommand && actIRCommand != lastIRCommand)
  {
    // todo: update Mode of operation
    switch (actIRCommand)
    {
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
      Go(20, 150);
      break;
    case 8:
      Go(20, -150);
      break;
    case 4: // 90degree left
      Turn(-DEG90, 150);
      break;
    case 6: // 90degree right
      Turn(DEG90, 150);
      break;
    case 1: // 45degree left
      Turn(-DEG45, 150);
      break;
    case 3: // 45degree right
      Turn(DEG45, 150);
      break;
    case 7: // 135degree left
      Turn(-DEG135, 150);
      break;
    case 9: // 135degree left
      Turn(DEG135, 150);
      break;
    case 33:
      doSquare(20, DEG90);
      break;
    case 40:
      doSpiral(20, 2, DEG90);
      break;
    case 16:
      doNikolaus(20);
      break;
    case 41:
      doLMR();
      break;
    case 19:
      break;
    case 20 :
      break;
    }
    lastIRCommand = actIRCommand ;
  }
//  else
//    lastIRCommand = 0;
}

void updateActors()
{
//  microM.Motors(leftSpeed,rightSpeed,leftBrake,rightBrake);    // update motor controller
}

void doProcessing()
{
}

void doSquare(int length, int turn)
{
  Pen(PEN_DOWN);
  for(int i=0; i<4; i++)
  {
    Go(length,150);
    Turn(turn,150);
  }
  Pen(PEN_UP);
}

void doSpiral(int limit, int steps, int angle)
{
  Pen(PEN_DOWN);
  for(int i=steps; i<=limit; i+=steps)
  {
    Go(i,150);
    Turn(angle,150);
    Go(i,150);
    Turn(angle,150);
  }
  Pen(PEN_UP);
}

void doNikolaus(int length)
{
  int diagonale = int((float)length*1.41F);
  int roof = int((float)length*0.707F);

  Pen(PEN_DOWN);
  Go(length,150);
  Turn(DEG45,150);
  Go(roof,150);
  Turn(DEG90,150);
  Go(roof,150);
  Turn(DEG45,150);
  Go(length,150);
  Turn(DEG135,150);
  Go(diagonale,150);
  Turn(DEG135,150);
  Go(length,150);
  Turn(DEG135,150);
  Go(diagonale,150);
  Turn(-DEG135,150);
  Go(length,150);
  Pen(PEN_UP);
}

void doWMR()
{
  Pen(PEN_UP);
  Go(20,150);
  Turn(DEG180,150); 
// draw W
  Pen(PEN_DOWN);
  Go(20,150);
  Turn(-DEG135,150);
  Go(10,150);
  Turn(DEG90,150);
  Go(10,150);
  Turn(-DEG135,150);
  Go(20,150);
  Pen(PEN_UP);

  Turn(DEG180,150);
  Go(20,150);
  Turn(-DEG90,150);
  Go(5,150);
  Turn(DEG90,150);

// draw M
  Pen(PEN_DOWN);
  Go(20,150);
  Turn(-DEG135,150);
  Go(10,150);
  Turn(DEG90,150);
  Go(10,150);
  Turn(-DEG135,150);
  Go(20,150);
  Pen(PEN_UP);

  Turn(-DEG90,150);
  Go(5,150);
  Turn(-DEG90,150);
    
// draw R
  Pen(PEN_DOWN);
  Go(20,150);
  Turn(DEG90,150);
  Go(5,150);
  Turn(DEG45,150);
  Go(5,150);
  Turn(DEG45,150);
  Go(5,150);
  Turn(DEG45,150);
  Go(5,150);
  Turn(DEG45,150);
  Go(5,150);
  Turn(-DEG135,150);
  Go(10,150);
}

void doLMR()
{
// draw L
  Pen(PEN_DOWN);
  Go(20,150);
  Turn(-DEG90,150);
  Go(10,150);
  Pen(PEN_UP);
  Go(5,150);
  Turn(-DEG90,150);
// draw M
  Pen(PEN_DOWN);
  Go(20,150);
  Turn(DEG135,150);
  Go(10,150);
  Turn(-DEG90,150);
  Go(10,150);
  Turn(DEG135,150);
  Go(20,150);

  Pen(PEN_UP);
  Turn(-DEG90,150);
  Go(5,150);
  Turn(-DEG90,150);
    
// draw R
  Pen(PEN_DOWN);
  Go(20,150);
  Turn(DEG90,150);
  Go(4,150);
  Turn(DEG45,150);
  Go(4,150);
  Turn(DEG45,150);
  Go(4,150);
  Turn(DEG45,150);
  Go(4,150);
  Turn(DEG45,150);
  Go(4,150);
  Turn(-DEG135,150);
  Go(12,150);
}

void setup() 
{
  Serial.begin(38400);
  Serial.println("MiniMURCS v0.1");
  Serial.println("resetting!");

  microM.Setup(); 

  initEncoder();

  penServo.attach(penServoPin);
  penServo.write(penDnPos);
  delay(2000);
  penServo.write(penUpPos);
  delay(2000);
}


void loop()
{
  readSensors();
  doProcessing();
  updateActors();
}

ISR(ADC_vect)
{
  static unsigned char tmp[2],flag[2],toggle=FALSE;
  if (autoencode)
  {
    tmp[toggle]= ADCH;
    if (toggle) ADMUX = (1 <<ADLAR) | (1 <<REFS0) | encoderRtPin; 
    else ADMUX = (1 <<ADLAR) | (1 <<REFS0) | encoderLtPin; 
    
    encRaw[toggle] = tmp[toggle];

    if ( (tmp[toggle] < 70) && (flag[toggle] == TRUE)) 
    {
      encoder[toggle] ++;
      flag[toggle] = FALSE;
    }
    if ( (tmp[toggle] > 110) && (flag[toggle] == FALSE)) 
    {
      encoder[toggle] ++;
      flag[toggle] = TRUE; 
    }
    toggle ^= 1; 
  }
}




