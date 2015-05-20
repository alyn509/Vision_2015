#include <elapsedMillis.h>
#include <Stepper.h>
#include <LiquidCrystal.h>
#include <Servo.h> 
#include "VisionDC.h"
#include "VisionEncoders.h"
#include "VisionBase.h"
#include "VisionState.h"
#include "pins.h"
#include "constants.h"

#define NINETYSECONDS 89000L

VisionBase base;
elapsedMillis timeUpTimer, enemyTimer;
boolean stoppedEverything = true; 
VisionState state;
VisionSensor startButton;

void setup()
{   
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);    // side sensor pair
  pinMode(A0, OUTPUT);
  digitalWrite(A0, LOW);    // start sensor pair

  startButton.initPin(startButtonPin);
  startButton.setAsPullup();
  while(startButton.detect());
  Serial.begin(115200);
  timeUpTimer = 0;
  delay(50);
  base.init();
  stoppedEverything = false;
}

void loop()
{   
  switch (state)
  {  
    case 0:      
      base.update();
      state.waitMicros(10000, 0);
      break;
    default:
      state.doLoop();
  }   
  
  if(!stoppedEverything)
  {
    base.doLoop();        
    checkForObstacle();
  }
  testIfTimeUp();
}

void testIfTimeUp()
{
  if(timeUpTimer >= NINETYSECONDS)
    timeIsUpStopEverything();
}

void timeIsUpStopEverything()
{
  stoppedEverything = true;
  state = STATE_STOP;
  base.stopNow();
}

void checkForObstacle()
{
  if(!base.isStopped)
  {
    if(base.detected() == true)
      enemyTimer = 0;
    if(base.detected() == true && !base.isPaused && !base.ignoredSensors)   
      base.pause();
    else if(base.detected() == false && base.isPaused && enemyTimer > 500L)
      base.unpause();
  }
}
