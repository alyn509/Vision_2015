  #ifndef VisionDC_h
  #define VisionDC_h
  
  #include "Arduino.h"
  
  
  class VisionDC {
    public:
      int forwardPin, backwardPin;
      bool isOn = false;
    public:
      void init(int fwPin, int bwPin);
      void moveForward(unsigned char pwm);
      void moveBackward(unsigned char pwm);
      void stopMotor();
  };
  
  #endif

