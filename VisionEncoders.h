#ifndef VisionEncoders_h
#define VisionEncoders_h

#include "Arduino.h"
#include <elapsedMillis.h>

class VisionEncoders {
  public:    
     double currentPosition = 0;
     int lastState = LOW;
     int stepPin, bpin;
  public:
      void init(int pinStep, int bpin);
      double getPosition();  
      void updatePosition();
      void reset();
};

#endif
