  #ifndef VisionStepper_h
  #define VisionStepper_h
  
  #include "Arduino.h"
  #include <elapsedMillis.h>
  #include "VisionState.h"
  
  class VisionStepper {
    public:
      void init();
      void initPins(int enablePin, int directionPin, int stepPin);
      void initDirectionForward(boolean forward);
      void initDelays(unsigned long startSpeedDelay, unsigned long highPhaseDelay, unsigned long maxSpeedDelay, unsigned long pauseSpeedDelay);
      void initSizes(float wheelDiameter, int wheelRevolutionSteps, float distanceBetweenWheels);
      void initStepCmRatio(float stepCmRatio);
      void doLoop();
      void toggleDirection();
      void setDirectionForward();
      void setDirectionBackward();
      void setSpeed(double speed);
      boolean isOff();
      boolean isPaused();
      void stopNow();
      void start();
      void pause();
      void unpause();
    private:
      void doSetup();
      float computeSpeed();
    private:
      int enablePin, directionPin, stepPin;
      int enablePinState, directionPinState, stepPinState;
      boolean forwardDirection;
      VisionState stepState;
      float currentDelay, highPhaseDelay, delayBeforeTurnOff;
      
      float stepCmRatio; // steps for a cm
      float degreeStepRatio; //steps for a degree turn;
  };
  
  #endif

