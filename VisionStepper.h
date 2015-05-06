  #ifndef VisionStepper_h
  #define VisionStepper_h
  
  #include "Arduino.h"
  #include <elapsedMillis.h>
  #include "VisionState.h"
  
  #define CLASSIC_TACTIC 0
  #define AGGRESSIVE_TACTIC 40
  #define FAST_START 150
  #define FRIENDLY_TACTIC 80
  #define GREEDY_TACTIC 200
  #define HOMOLOGATION -200
  
  class VisionStepper {
    public:
      void init();
      void initPins(int enablePin, int directionPin, int stepPin);
      void initDirectionForward(boolean forward);
      void initDelays(unsigned long startSpeedDelay, unsigned long highPhaseDelay, unsigned long maxSpeedDelay, unsigned long pauseSpeedDelay);
      void setTacticDelays(int tactic);
      void initSizes(float wheelDiameter, int wheelRevolutionSteps, float distanceBetweenWheels);
      void initStepCmRatio(float stepCmRatio);
      void doLoop();
      void toggleDirection();
      void setDirectionForward();
      void setDirectionBackward();
      void setTargetDelay(unsigned long targetDelay);
      boolean isOff();
      boolean isPaused();
      boolean isAtTargetSpeed();
      void doSteps(unsigned long stepNumber);
      void doDistanceInCm(float distance);
      void doRotationInAngle(float angle);
      void stopNow();
      void setMaxSpeed();
      void pause();
      void unpause();
      void setSpecial();
      void resetSpecial();
      float getDistanceMadeSoFar();
      float getDistanceRemainedToDo();
    private:
      void doSetup();
      float computeSpeed();
    private:
      int enablePin, directionPin, stepPin;
      int enablePinState, directionPinState, stepPinState;
      boolean forwardDirection;
      VisionState motorState, enableState, speedState, stepState;
      long stepsMadeSoFar, stepsRemaining;
      float stepSpeedCounter, stepSpeedCounterAcceleration, stepSpeedCounterSlowing;
      float startSpeedDelay, currentDelay, targetDelay, pauseSpeedDelay, delayBeforeTurnOff, highPhaseDelay, savedWhenPausingDelay, savedDeacceleration;
      int old_state;
      elapsedMicros stepTimer;
      elapsedMillis stopTimer, pauseTurnOff;
      boolean special;
      
      float stepCmRatio; // steps for a cm
      float degreeStepRatio; //steps for a degree turn;
  };
  
  #endif

