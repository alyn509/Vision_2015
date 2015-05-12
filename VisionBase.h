#ifndef VisionBase_h
#define VisionBase_h

#include "Arduino.h"
#include "VisionStepper.h"
#include "VisionSensor.h"
#include "VisionEncoders.h"
#include "pins.h"
#include "constants.h"
#include <elapsedMillis.h>
#include <Servo.h>

#define NONE 4
#define FRONT 3
#define BACK 0
#define LEFT 1
#define RIGHT 2

class VisionBase {
  public:
    void init();
    void setStartDelays(unsigned long startDelay);
    void setTacticDelays(int tactic);
    
    void moveForward(float distance, unsigned long step_delay);
    void moveBackward(float distance, unsigned long step_delay);
    
    boolean frontDetected();
    
    void turnLeft(int angle);
    void turnRight(int angle);
    
    void doMovementRequirements(int step_delay);
    
    void pause();
    void unpause();
    void stopNow();
    void doLoop();
    
    bool leftMotorDir();
    bool rightMotorDir();
    
    void setSpecial();
    void resetSpecial();
    
    boolean isStopped();
    boolean isPaused();
    
    float encoderValue(float value);    
    
    void update();
    
  public:
    VisionStepper leftMotor, rightMotor;
    VisionEncoders leftEncoder, rightEncoder;
    
    VisionState state;
    VisionSensor frontSensor;
    
    int directionMovement;
    
    bool ignoredSensors;
    
    float lastPositionLeft = 0;
    float lastPositionRight = 0;
};

#endif

