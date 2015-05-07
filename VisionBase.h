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
#include <PID_v1.h>

#define NONE 4
#define FRONT 3
#define BACK 0
#define LEFT 1
#define RIGHT 2

class VisionBase {
  public:
    void init();
    Servo sensorScanner;
    elapsedMillis sensorToggleTimer;
    
    void moveForward(float distance, unsigned long step_delay);
    void moveBackward(float distance, unsigned long step_delay);
    
    boolean frontDetected();
    
    void checkObstructions();
    void turnLeft(int angle);
    void turnRight(int angle);
    
    void pause();
    void unpause();
    void stopNow();
    void doLoop();
    
    boolean isStopped();
    boolean isPaused();
    
    float getDistanceMadeSoFar();
    float encoderValue(float value);    
    
    void update();
    
  public:
    VisionStepper leftMotor, rightMotor;
    VisionEncoders leftEncoder, rightEncoder;
    
    VisionSensor frontSensor;
    

    
    boolean ignoredSensors;
    boolean obstructionDetected;
};

#endif

