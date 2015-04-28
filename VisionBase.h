#ifndef VisionBase_h
#define VisionBase_h

#include "Arduino.h"
#include "VisionStepper.h"
#include "VisionSensor.h"
#include "pins_little_robot.h"
#include "little_robot_constants.h"
#include <elapsedMillis.h>
#include <Servo.h>

#define NONE 0
#define FRONT 1
#define BACK 2
#define LEFT 3
#define RIGHT 4

class VisionBase {
  public:
    void init();
    Servo sensorScanner;
    elapsedMillis sensorToggleTimer;
    void setStartDelays(unsigned long startDelay);
    void setTacticDelays(int tactic);
    void moveForward(float distance, unsigned long step_delay);
    void moveBackward(float distance, unsigned long step_delay);
    boolean frontDetected();
    //boolean leftDetected();
    //boolean rightDetected();
    boolean backDetected();
    void checkObstructions();
    void turnLeft(int angle);
    void turnRight(int angle);
    void pause();
    void unpause();
    void stopNow();
    void doLoop();
    void setSpecial();
    void resetSpecial();
    boolean isStopped();
    boolean isPaused();
    float getDistanceMadeSoFar();
  public:
    VisionStepper leftMotor, rightMotor;
    VisionSensor backLeft, backRight, backLow, frontLeft, frontFront, frontRight;
    int directionMovement;
    boolean oppositeSide;
    boolean ignoredSensors;
    boolean obstructionDetected;
};

#endif

