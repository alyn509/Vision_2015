#ifndef VisionEncoders_h
#define VisionEncoders_h

#include "Arduino.h"
#include <elapsedMillis.h>

class VisionEncoders {
  public:    
     long currentPosition = 0;
     long lastReadPosition = 0;
     int lastState = LOW;
     int stepPin;
  public:
      void init(int pinStep);
      long getPosition();  
      void updatePosition(bool dir);  // dir = 1 - forward |  dir = 0 - backward
      int getTraveledLength();
};

#endif
