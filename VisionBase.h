#ifndef VisionBase_h
#define VisionBase_h

#include "Arduino.h"
#include "VisionStepper.h"
#include "VisionSensor.h"
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
    
    void setSpecial();
    void resetSpecial();
    
    boolean isStopped();
    boolean isPaused();
    
    void attachServoz();
    
    /************************************************** Servoz ***************************************************/
    
    void openLeftArm();     void closeLeftArm();     void grabLeftArm();
    void openRightArm();    void closeRightArm();    void grabRightArm();
    
    void openLeftClaw();     void closeLeftClaw();     void grabLeftClaw();
    void openRightClaw();    void closeRightClaw();    void grabRightClaw();
    
    void holdLeftLimitator();     void releaseLeftLimitator();
    void holdRightLimitator();    void releaseRightLimitator();

    void openLeftDoow();     void closeLeftDoor();
    void openRightDoor();    void closeRightDoor();
    
    void releaseLeftPopcorn();
    void releaseRightPopcorn();
    
    void gatherPopcorn();
    void stopGatherPopcorn();
    
    void riseLift();
    void lowerLift();
    void stopLift();
    
    /******************************************************************************************************/
        
  public:
    VisionStepper leftMotor, rightMotor;
    
    Servo leftClaw, rightClaw,
          leftLimitator, rightLimitator,
          leftPopcornHolder, rightPopcornHolder,
          leftArm, rightArm,
          leftDoor, rightDoor;
          
    VisionSensor frontLeftSensor, frontRightSensor, liftLimitatorSensor;
    
    int directionMovement;
    
    bool ignoredSensors;
    bool goingUp = false;
};

#endif

