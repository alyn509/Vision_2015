#ifndef VisionSensorsDevices_h
#define VisionSensorsDevices_h

#include "Arduino.h"
#include "VisionSensor.h"
#include <elapsedMillis.h>
#include <Servo.h>

class VisionDevices {
  public:    
    void init();
    void ThrowNet();
    void initNet();
    void shootBall();
    void startShooting();
    void startSpinningBallTray();
    void stopSpinningBallTray();
    void stopShooting();
    void pauseSpinningBallTray();
    void resumeSpinningBallTray();
  public:
    Servo net;
    boolean shooting;
};

#endif
