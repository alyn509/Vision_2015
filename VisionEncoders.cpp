#include "VisionEncoders.h"
#include "pins.h"
#include <elapsedMillis.h>

void VisionEncoders::init(int pinStep, int bpin) 
{
   stepPin = pinStep;
   this->bpin = bpin;
   pinMode(stepPin,INPUT_PULLUP);
   pinMode(bpin,INPUT_PULLUP);
}

double VisionEncoders::getPosition() 
{ 
   return currentPosition;
} 

void VisionEncoders::updatePosition() 
{ 
   int currentState = digitalRead(stepPin);
   int bvalue = digitalRead(bpin);
   if (lastState != currentState)
   {
       currentPosition += (currentState ^ bvalue) ? -1 : 1;
       lastState = currentState;
   }
}

void VisionEncoders::reset()
{
  currentPosition = 0;
}

