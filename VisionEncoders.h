#ifndef VisionEncoders_h
#define VisionEncoders_h

#include "Arduino.h"
#include <elapsedMillis.h>

class VisionEncoders {
  public:    
     long currentPossition = 0;
     int lastState = LOW;
     int stepPin;
     int directionPin;
  public:
      void init(int pinStep,int pinDirection);
      long getPossition();
};

#endif
