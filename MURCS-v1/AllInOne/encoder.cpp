#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#include "iomapping.h"
#include "encoder.h"

volatile int encoder[2];
volatile int encRaw[2];
int encMin[2];
int encMax[2];
int encFlag[2];
volatile int autoencode=false;

void stopEncoder(void) 
{ 
  autoencode=false; 
}

void startEncoder(void) 
{ 
  autoencode=true; 
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
  autoencode=true;
  sei();
  setEncoder(0,0);
}

void readEncoder(int debugLevel)
{
  if (autoencode)
  {
    if (debugLevel > 1) 
    {
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
    }
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
        if (debugLevel > 1) 
        {
          Serial.print("Enc: ");
          Serial.print(i, DEC);
          Serial.print(" ");
          Serial.println(encoder[i], DEC);
        }
      }
      if (encRaw[i] > 450 && encFlag[i] == 0)
      {
        encoder[i]++;
        encFlag[i] = 1;
        if (debugLevel > 1) 
        {
          Serial.print("Enc: ");
          Serial.print(i, DEC);
          Serial.print(" ");
          Serial.println(encoder[i], DEC);
        }
      }
      if (encRaw[i] < encMin[i]) 
      {
        encMin[i] = encRaw[i];
        if (debugLevel > 2) 
        {
          Serial.print("Min: ");
          Serial.print(i, DEC);
          Serial.print(" ");
          Serial.println(encMin[i], DEC);
        }
      }
      if (encRaw[i] > encMax[i]) 
      {
        encMax[i] = encRaw[i];
        if (debugLevel > 2) 
        {
          Serial.print("Max: ");
          Serial.print(i, DEC);
          Serial.print(" ");
          Serial.println(encMax[i], DEC);
        }
      }
    }
  }

}


ISR(ADC_vect)
{
  static unsigned char tmp[2],flag[2],toggle=false;
  if (autoencode)
  {
    tmp[toggle]= ADCH;
    if (toggle) ADMUX = (1 <<ADLAR) | (1 <<REFS0) | encoderRtPin; 
    else ADMUX = (1 <<ADLAR) | (1 <<REFS0) | encoderLtPin; 
    
    encRaw[toggle] = tmp[toggle];

    if ( (tmp[toggle] < 70) && (flag[toggle] == true)) 
    {
      encoder[toggle] ++;
      flag[toggle] = false;
    }
    if ( (tmp[toggle] > 110) && (flag[toggle] == false)) 
    {
      encoder[toggle] ++;
      flag[toggle] = true; 
    }
    toggle ^= 1; 
  }
}




 

