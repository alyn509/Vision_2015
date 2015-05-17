#include "VisionBase.h"

void VisionBase::init()
{
  frontLeftSensor.initPin(frontLeftSensorPin);
  frontRightSensor.initPin(frontRightSensorPin);
  
  liftLimitatorSensor.initPin(liftLimitatorSensorPin);
  liftLimitatorSensor.setAsPullup();
    
  leftMotor.init();
  leftMotor.initDirectionForward(HIGH);
  leftMotor.initPins(leftMotorEnablePin, leftMotorDirectionPin, leftMotorStepPin);
  leftMotor.initDelays(defaultStartSpeedDelay, highPhaseDelay, pauseSpeedDelay, delayBeforeTurnOff);
  leftMotor.initSizes(wheelDiameter, wheelRevolutionSteps,distanceBetweenWheels);    
  
  rightMotor.init();
  rightMotor.initDirectionForward(LOW);
  rightMotor.initPins(rightMotorEnablePin, rightMotorDirectionPin, rightMotorStepPin);
  rightMotor.initDelays(defaultStartSpeedDelay, highPhaseDelay, pauseSpeedDelay, delayBeforeTurnOff);
  rightMotor.initSizes(wheelDiameter, wheelRevolutionSteps,distanceBetweenWheels);   
    
  pinMode(popCornGrabberDCPin, OUTPUT);
  pinMode(upLiftPin, OUTPUT);
  pinMode(downLiftPin, OUTPUT);
  
  attachServoz();

  directionMovement = NONE;
  ignoredSensors = false;
}
void VisionBase::attachServoz()
{
  leftClaw.attach(leftClawServoPin);                  closeLeftClaw();
  rightClaw.attach(rightClawServoPin);                closeRightClaw();
  
  leftLimitator.attach(leftLimitatorServoPin);        releaseLeftLimitator();
  rightLimitator.attach(rightLimitatorServoPin);      releaseRightLimitator();
  
  leftPopcornHolder.attach(leftPopCornHolderPin);     leftPopcornHolder.write(180);
  rightPopcornHolder.attach(rightPopCornHolderPin);   rightPopcornHolder.write(0);
  
  leftArm.attach(leftArmServoPin);                    closeLeftArm();
  rightArm.attach(rightArmServoPin);                  closeRightArm();
  
  leftDoor.attach(leftDoorServoPin);
  rightDoor.attach(rightDoorServoPin);
}

void VisionBase::setTacticDelays(int tactic)
{
  rightMotor.setTacticDelays(tactic);
  leftMotor.setTacticDelays(tactic);
}

void VisionBase::setStartDelays(unsigned long startDelay)
{
  rightMotor.initDelays(startDelay, highPhaseDelay, pauseSpeedDelay, delayBeforeTurnOff);
  leftMotor.initDelays(startDelay, highPhaseDelay, pauseSpeedDelay, delayBeforeTurnOff);
}

void VisionBase::moveForward(float distance, unsigned long step_delay)
{       
  directionMovement = FRONT;
  
  doMovementRequirements(step_delay);  
  
  leftMotor.setDirectionForward();
  rightMotor.setDirectionForward(); 
  
  leftMotor.doDistanceInCm(distance);
  rightMotor.doDistanceInCm(distance);
}

void VisionBase::moveBackward(float distance, unsigned long step_delay)
{    
  directionMovement = BACK;   

  doMovementRequirements(step_delay);

  leftMotor.setDirectionBackward();
  rightMotor.setDirectionBackward();  
  
  leftMotor.doDistanceInCm(distance);
  rightMotor.doDistanceInCm(distance);
}

void VisionBase::turnLeft(int angle)
{
  directionMovement = LEFT; 

  doMovementRequirements(5000);
  
  leftMotor.setDirectionBackward();
  rightMotor.setDirectionForward();
  
  leftMotor.doRotationInAngle(angle);
  rightMotor.doRotationInAngle(angle); 
}

void VisionBase::turnRight(int angle)
{  
  directionMovement = RIGHT;
  
  doMovementRequirements(5000);
    
  leftMotor.setDirectionForward();
  rightMotor.setDirectionBackward();
  
  leftMotor.doRotationInAngle(angle);
  rightMotor.doRotationInAngle(angle);
  
}

void VisionBase::doMovementRequirements(int step_delay)
{
  leftMotor.setTargetDelay(step_delay);
  rightMotor.setTargetDelay(step_delay);
}

void VisionBase::setSpecial()
{
  leftMotor.setSpecial();
  rightMotor.setSpecial();
}

void VisionBase::resetSpecial()
{
  leftMotor.resetSpecial();
  rightMotor.resetSpecial();
}

void VisionBase::pause()
{ 
  leftMotor.pause();
  rightMotor.pause();
}
void VisionBase::unpause()
{
  leftMotor.unpause();
  rightMotor.unpause();
}

bool VisionBase::frontDetected()
{
  return frontLeftSensor.detect() || frontRightSensor.detect();
}

bool VisionBase::isStopped()
{
  return rightMotor.isOff();
}

bool VisionBase::isPaused()
{
  return rightMotor.isPaused();
}

void VisionBase::doLoop()
{
  leftMotor.doLoop();
  rightMotor.doLoop();
  if(liftLimitatorSensor.detect() && goingUp == true)
  {
    digitalWrite(upLiftPin, LOW);  
    goingUp = false;
  }
}

void VisionBase::stopNow()
{    
  leftMotor.stopNow();
  rightMotor.stopNow();
}
/********************************************************* Servoz *********************************************************/
void VisionBase::openLeftArm()
{
  leftArm.write(30);
}

void VisionBase::closeLeftArm()
{
  leftArm.write(110);
}

void VisionBase::grabLeftArm()
{
  leftArm.write(90);
}

void VisionBase::openRightArm()
{
  rightArm.write(100);
}

void VisionBase::closeRightArm()
{
  rightArm.write(20);
}

void VisionBase::grabRightArm()
{
  rightArm.write(40);
}
 
void VisionBase::openLeftClaw()
{
  leftClaw.write(150);
}

void VisionBase::closeLeftClaw()
{
  leftClaw.write(12);
}

void VisionBase::grabLeftClaw()
{
  leftClaw.write(40);
}

void VisionBase::openRightClaw()
{
  rightClaw.write(5);
}

void VisionBase::closeRightClaw()
{
  rightClaw.write(138);
}

void VisionBase::grabRightClaw()
{
  rightClaw.write(115);
}
    
void VisionBase::holdLeftLimitator()
{
  leftLimitator.write(13);
}

void VisionBase::releaseLeftLimitator()
{
  leftLimitator.write(40);
}

void VisionBase::holdRightLimitator()
{
  rightLimitator.write(97);
}

void VisionBase::releaseRightLimitator()
{
  rightLimitator.write(60);
}

void VisionBase::openLeftDoow()    // unimplemented
{
  leftDoor.write(180);
}

void VisionBase::closeLeftDoor()    // unimplemented
{
  leftDoor.write(0);
}

void VisionBase::openRightDoor()    // unimplemented
{
  rightDoor.write(0);
}

void VisionBase::closeRightDoor()    // unimplemented
{
  rightDoor.write(0);
}
  
void VisionBase::releaseLeftPopcorn()
{
  leftPopcornHolder.write(130);
}

void VisionBase::releaseRightPopcorn()
{
  rightPopcornHolder.write(60);
}
    
void VisionBase::gatherPopcorn()
{
  digitalWrite(popCornGrabberDCPin, HIGH);
}

void VisionBase::stopGatherPopcorn()
{
  digitalWrite(popCornGrabberDCPin, LOW);
}
    
void VisionBase::riseLift()
{
  if(!liftLimitatorSensor.detect())
  {
    digitalWrite(upLiftPin, HIGH);
    goingUp = true;
  }
  else
  {
    digitalWrite(upLiftPin, LOW);  
    goingUp = false;
  }
}

void VisionBase::lowerLift()
{
  digitalWrite(downLiftPin, HIGH);
}
void VisionBase::stopLift()
{
  digitalWrite(upLiftPin, LOW);
  digitalWrite(downLiftPin, LOW);
}
