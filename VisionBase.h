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
    
    void moveForward(double distance);
    void moveBackward(double distance);
    void turnLeft(double distance);
    void turnRight(double distance);
    void move(double distance);
    
    boolean frontDetected();
    
    void checkObstructions();
    
    void pause();
    void unpause();
    void stopNow();
    void doLoop();
    
    boolean isStopped();
    boolean isPaused();
    
    double encoderValue(double value);
    double distanceToEncoderTicks(double distance);
    
    void update();
    
  public:
    VisionStepper leftMotor, rightMotor;
    VisionEncoders leftEncoder, rightEncoder;
    
    VisionSensor frontSensor;
    
    VisionState state;
    
    boolean ignoredSensors;
    boolean obstructionDetected;
};

#endif

