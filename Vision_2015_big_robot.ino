#include <elapsedMillis.h>
#include <Stepper.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include <PID_v1.h>
#include "VisionStepper.h"
#include "VisionBase.h"
#include "VisionDevices.h"
#include "VisionState.h"
#include "VisionEncoders.h"
#include "pins.h"
#include "constants.h"

#define NINETYSECONDS 89000L

VisionBase base;
VisionDevices devices;
elapsedMillis timeUpTimer;
elapsedMillis extra;
elapsedMillis enemyTimer, enemyThere;
boolean stoppedEverything = true; 

VisionState state;
float distanceToDo = 0;

#define YELLOW_START_STATE 0
#define GREEN_START_STATE 200
#define HOMOLOGATION 400

void setup()
{ 
  //Serial.begin(115200);
  timeUpTimer = 0;
  base.init();
  state.wait(100, YELLOW_START_STATE);
  stoppedEverything = false;
}

void loop()
{
  switch (state)
  {
    case 0:
      base.update();
      state.waitMicros(10000,0);
      break;
    default:
      state.doLoop();
  }
  if(!stoppedEverything)
  {
    base.checkObstructions();
    base.doLoop();
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
  extra = 0;
  state = STATE_STOP;
  base.stopNow();
}

boolean baseStop()
{
  return base.isStopped();
}

