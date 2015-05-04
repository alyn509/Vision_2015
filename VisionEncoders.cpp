#include "VisionEncoders.h"
#include "pins.h"
#include <elapsedMillis.h>

void VisionEncoders::init(int pinStep) 
{ 
   stepPin = pinStep;
   pinMode (stepPin,INPUT_PULLUP);
} 

long VisionEncoders::getPosition() 
{ 
   return currentPosition;
} 

void VisionEncoders::updatePosition(bool dir) 
{ 
   int currentState = digitalRead(stepPin);
   if (lastState != currentState) 
   {
     if (dir == LOW) 
       currentPosition--;
     else
       currentPosition++;
   } 
   lastState = currentState;
} 

int VisionEncoders::getTraveledLength()
{
   int traveled = currentPosition - lastReadPosition;
   lastReadPosition = currentPosition;
   return traveled;
}
