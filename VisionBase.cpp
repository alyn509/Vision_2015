#include "VisionBase.h"

void VisionBase::init()
{
  frontLeftSensor.initPin(frontLeftSensorPin);
  frontRightSensor.initPin(frontRightSensorPin);
    
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
    
  directionMovement = NONE;
  ignoredSensors = false;
  
  state = 0;
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
}

void VisionBase::stopNow()
{    
  leftMotor.stopNow();
  rightMotor.stopNow();
}
