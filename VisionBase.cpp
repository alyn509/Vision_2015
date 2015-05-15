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
  /*
  leftEncoder.init(leftEncoderStepPin);
  rightEncoder.init(rightEncoderStepPin);*/
    
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
/*  
  lastPositionLeft = leftEncoder.getPosition();
  lastPositionRight = rightEncoder.getPosition();*/
}

bool VisionBase::leftMotorDir()
{
  return (directionMovement / 2) % 2;
}

bool VisionBase::rightMotorDir()
{  
  return directionMovement % 2;
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
/*    
  leftEncoder.updatePosition();
  rightEncoder.updatePosition();*/
}

void VisionBase::stopNow()
{    
  leftMotor.stopNow();
  rightMotor.stopNow();
}
/*
float VisionBase::encoderValue(float value)
{
  return (value * PI * wheelDiameter) / encoderResolution;
}*/
/*
void VisionBase::update()
{  
  int admittedError = 5;
  int leftError = encoderValue(lastPositionLeft - leftEncoder.getPosition()) - leftMotor.getDistanceMadeSoFar();
  int rightError = encoderValue(lastPositionRight - rightEncoder.getPosition()) - rightMotor.getDistanceMadeSoFar();
  
  if (abs(leftError) > admittedError)
  {
    leftEncoder.currentPosition = 0;
    leftMotor.doDistanceInCm(leftMotor.getDistanceRemainedToDo() + leftError);
  }
  if (abs(rightError) > admittedError)
  {
    rightEncoder.currentPosition = 0;
    rightMotor.doDistanceInCm(rightMotor.getDistanceRemainedToDo() + rightError);
  }
}*/
