#ifndef VisionBase_h
#define VisionBase_h

#include "Arduino.h"
#include "VisionDC.h"
#include "VisionSensor.h"
#include "VisionState.h"
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

#define PAUSED 500
#define OVER 501
#define HOMOLOGATION 400
#define YELLOW_SIDE 0
#define GREEN_SIDE 100

class VisionBase {
  public:
    void init();
    
    void moveForward(int distance,int pwm, int nextState);
    void moveBackward(int distance,int pwm, int nextState);
    
    boolean frontDetected();
    
    void turnLeft(int angle,int pwmv, int nextState);
    void turnRight(int angle,int pwmv, int nextState);
    
    float cmToSteps(float value);
    float angleToSteps(float value);
    
    void doDistanceInCM(int dist, int nextState);
    void doAngleRotation(int dist, int nextState);
    
    void pause();
    void unpause();
    
    void stopNow();
    
    void doLoop();
    void update();
            
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
    VisionDC leftMotor, rightMotor;
    VisionEncoders leftEncoder, rightEncoder;
    VisionState state;
    
    Servo leftClaw, rightClaw,
          leftLimitator, rightLimitator,
          leftPopcornHolder, rightPopcornHolder,
          leftArm, rightArm,
          leftDoor, rightDoor;
          
    VisionSensor frontLeftSensor, frontRightSensor, liftLimitatorSensor;
    
    int pwmValue = 0;
    int directionMovement;
    int stateBeforePause;
    float lastPositionLeft = 0;
    float lastPositionRight = 0;
    
    bool isPaused = false;
    bool isResuming = false;
    bool isStopped = false;
    bool ignoredSensors = false;
    bool newMovement = false;
    bool goingUp = false;
};

#endif

