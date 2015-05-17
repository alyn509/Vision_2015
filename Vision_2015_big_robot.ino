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
  digitalWrite(3, LOW);    // we set pin 3 as GND.. because of Claudiu
  
  startButton.initPin(startButtonPin);
  startButton.setAsPullup();
  while(startButton.detect());
  
  Serial.begin(115200);
  timeUpTimer = 0;
  base.init();
  delay(50);
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
  if(timeUpTimer == NINETYSECONDS)
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
    if(base.frontDetected() == true ) 
      enemyTimer = 0;
    if(base.frontDetected() == true && !base.isPaused && !base.ignoredSensors)   
      base.pause();
    else if(base.frontDetected() == false && base.isPaused && enemyTimer > 500L)
      base.unpause();
  }
}
