#include <elapsedMillis.h>
#include <Stepper.h>
#include <LiquidCrystal.h>
#include <TimerThree.h>
#include <Servo.h> 
#include "VisionStepper.h"
#include "VisionBase.h"
#include "VisionState.h"
#include "pins.h"
#include "constants.h"

#define NINETYSECONDS 89000L

VisionBase base;
elapsedMillis timeUpTimer, enemyTimer;
boolean stoppedEverything = true; 

VisionState movementState;

VisionSensor startButton;
void setup()
{   
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);    // we set pin 3 as GND.. because of Claudiu
  startButton.initPin(startButtonPin);
  startButton.setAsPullup();
  while(startButton.detect());
  delay(50);
  Serial.begin(115200);
  timeUpTimer = 0;
  base.init();
  stoppedEverything = false;
    
  base.setTacticDelays(CLASSIC_START);
}

void loop()
{  
  if(!stoppedEverything)
  {
    base.doLoop();
     
    switch(movementState)
    {
      case 0: 
        base.moveForward(45, 4000);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 1:
        base.turnRight(90);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 2: 
        base.openLeftArm();
        base.moveForward(50, 4000);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 3:
        base.turnRight(90);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 4: 
        base.moveForward(30, 4000);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 5:
        base.grabLeftArm();
        movementState.wait(300,STATE_NEXT);
        break;
      case 6: 
        base.moveForward(3, 4000);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 7: 
        base.turnLeft(80);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 8: 
        base.moveForward(5, 4000);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 9: 
        base.grabRightClaw();
        movementState.wait(100,STATE_STOP);
        break;
      case 10: 
        base.turnLeft(10);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case STATE_STOP:
        break;
      default:
        movementState.doLoop();   
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
  movementState = STATE_STOP;
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
