#include "VisionEncoders.h"
#include "pins.h"
#include <elapsedMillis.h>

void VisionEncoders::init(int pinStep,int pinDirection) 
{ 
   stepPin = pinStep;
   directionPin = pinDirection;
   pinMode (stepPin,INPUT);
   pinMode (directionPin,INPUT);
} 

long VisionEncoders::getPossition() 
{ 
   int currentState = digitalRead(stepPin);
   if ((lastState == LOW) && (currentState == HIGH)) 
   {
     if (digitalRead(directionPin) == LOW) 
       currentPossition--;
     else
       currentPossition++;
   } 
   lastState = currentState;
   return currentPossition;
} 
