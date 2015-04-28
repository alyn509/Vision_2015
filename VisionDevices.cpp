  #include "VisionDevices.h"
#include "pins_little_robot.h"
#include <elapsedMillis.h>

elapsedMillis waitTime;

//boolean FirstBallFlag = true;
     
const int delayActions = 4000;
const int delayBallShotsTogglesHigh = 10;
const int delayBallShotsTogglesLow = 40;
//const int ballToggleTimes = 9999999;
//int toggleCounter;
    
void VisionDevices::init()
{ 
  //FirstBallFlag = true;
  //toggleCounter = 0;
  pinMode(PrepareBallPin, OUTPUT);
  digitalWrite(PrepareBallPin, LOW);
  
  pinMode(ShootBallPin, OUTPUT);
  digitalWrite(ShootBallPin, LOW);
  
  net.attach(ThrowNetPin);
  initNet();
  shooting = false;
}

void VisionDevices::shootBall()
{
  //if(toggleCounter<=ballToggleTimes){
    if(waitTime>delayBallShotsTogglesHigh){
      digitalWrite(ShootBallPin, HIGH);
    }
    if(waitTime>(delayBallShotsTogglesHigh + delayBallShotsTogglesLow))
    {
      digitalWrite(ShootBallPin, LOW);
      //toggleCounter++;
      waitTime = 0;
    }
  //}
}

void VisionDevices::startShooting()
{
  digitalWrite(ShootBallPin, HIGH);
}

void VisionDevices::stopShooting()
{
   digitalWrite(ShootBallPin, LOW);
}
void VisionDevices::pauseSpinningBallTray()
{
  if(shooting == true)
    digitalWrite(PrepareBallPin, LOW);
}

void VisionDevices::resumeSpinningBallTray()
{
  if(shooting == true)
   digitalWrite(PrepareBallPin, HIGH);
}

void VisionDevices::startSpinningBallTray()
{
  digitalWrite(PrepareBallPin, HIGH);
  shooting = true;
  waitTime = 0;
}

void VisionDevices::stopSpinningBallTray()
{
  digitalWrite(PrepareBallPin, LOW);
  shooting = false;
}

void VisionDevices::ThrowNet()
{
  net.write(0);
}

void VisionDevices::initNet()
{
  net.write(55);
}
