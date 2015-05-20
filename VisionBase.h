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
    boolean backDetected();
    boolean detected();
    
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
    int getState(int stateToGet, int originalState);
    
    /************************************************** Servoz ***************************************************/
    
    void openLeftArm();     void closeLeftArm();     void grabLeftArm();
    void openRightArm();    void closeRightArm();    void grabRightArm();
    void openArm();         void closeArm();         void grabArm();
    void openOtherArm();    void closeOtherArm();    void grabOtherArm();
    
    void openClaw();     void closeClaw();
    void openDoor();     void closeDoor();   void unlockDoor();
    
    void releaseLeftPopcorn();
    void releaseRightPopcorn();
    
    void gatherPopcorn();
    void stopGatherPopcorn();
    
    void riseLift(int stateNext);
    void halfLowerLift(int stateNext);
    void lowerLift(int stateNext);
    void stopLift(int stateNext);
    
    /******************************************************************************************************/
        
  public:
    VisionDC leftMotor, rightMotor;
    VisionEncoders leftEncoder, rightEncoder;
    VisionState state, deviceState;
    
    Servo claw, door,
          leftPopcornHolder, rightPopcornHolder,
          leftArm, rightArm;
          
    VisionSensor frontLeftSensor, frontRightSensor, backLeftSensor, backMidSensor, backRightSensor, liftLimitatorSensor, sideButton;
    
    int pwmValue = 0;
    int directionMovement;
    int stateBeforePause;
    int sideGreen;
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

