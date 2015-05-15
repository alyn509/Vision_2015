#include <elapsedMillis.h>
#include <Stepper.h>
#include <LiquidCrystal.h>
#include <TimerThree.h>
#include <Servo.h> 
#include "VisionStepper.h"
#include "VisionBase.h"
#include "VisionState.h"
#include "VisionEncoders.h"
#include "pins.h"
#include "constants.h"

#define NINETYSECONDS 89000L

VisionBase base;
elapsedMillis timeUpTimer, enemyTimer;
boolean stoppedEverything = true; 

VisionState state, movementState;

VisionSensor startButton;
void setup()
{   
  startButton.initPin(startButtonPin);
  startButton.setAsPullup();
//  while(startButton.detect());
  delay(50);
  Serial.begin(115200);
  timeUpTimer = 0;
  base.init();
  stoppedEverything = false;
    
  base.setTacticDelays(CLASSIC_START);
}

void loop()
{ 
  switch (state)
  {  
    default:
      state.doLoop();       
    //  base.update();
  }
   
  
  if(!stoppedEverything)
  {
    base.doLoop();
     
    switch(movementState)
    {
      case 0: 
        base.moveForward(30, 1000);
        movementState.waitFor(baseStop,1);
        break;
      case 1:
        base.moveBackward(30, 1000);
        movementState.waitFor(baseStop,0);
        break;
      case STATE_STOP:
        break;
    }
        
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
  if(!base.isStopped())
  {
    if(base.frontDetected() == true ) 
      enemyTimer = 0;
    if(base.frontDetected() == true && !base.isPaused() && !base.ignoredSensors)   
      base.pause();
    else if(base.frontDetected() == false && base.isPaused() && enemyTimer > 500L)
      base.unpause();
  }
}

boolean baseStop()
{
  return base.isStopped();
}
