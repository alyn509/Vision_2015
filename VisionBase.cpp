#include "VisionBase.h"

void VisionBase::init()
{
  frontSensor.initPin(frontSensorPin);
    
  leftMotor.initDirectionForward(HIGH);
  leftMotor.initPins(leftMotorEnablePin, leftMotorDirectionPin, leftMotorStepPin);
  
  rightMotor.init();
  rightMotor.initDirectionForward(LOW);
  rightMotor.initPins(rightMotorEnablePin, rightMotorDirectionPin, rightMotorStepPin);
  rightMotor.initSizes(wheelDiameter, wheelRevolutionSteps,distanceBetweenWheels);    
  
  leftEncoder.init(leftEncoderStepPin, leftEncoderBPin);
  rightEncoder.init(rightEncoderStepPin, rightEncoderBPin);
  
  obstructionDetected = false;
  ignoredSensors = false;
}

void VisionBase::moveForward(float distance, unsigned long step_delay)
{
  rightMotor.setTargetDelay(step_delay);
  leftMotor.setDirectionForward();
  rightMotor.setDirectionForward();
  rightMotor.doDistanceInCm(distance);
  leftEncoder.reset();
  rightEncoder.reset();
}

void VisionBase::moveBackward(float distance, unsigned long step_delay)
{
}

void VisionBase::turnLeft(int angle)
{
}

void VisionBase::turnRight(int angle)
{
}

void VisionBase::pause()
{
  rightMotor.pause();
}

void VisionBase::unpause()
{
  rightMotor.unpause();
}

boolean VisionBase::frontDetected()
{
  return frontSensor.detect();
}

boolean VisionBase::isStopped()
{
  return rightMotor.isOff();
}

boolean VisionBase::isPaused()
{
  return rightMotor.isPaused();
}

void VisionBase::checkObstructions()
{
  obstructionDetected = false;
  if (frontDetected() && !ignoredSensors)
    obstructionDetected = true;
}

double turnErrorInput, turnOutput, turnErrorTarget;
PID turnPID(&turnErrorInput, &turnOutput, &turnErrorTarget,2,5,1, DIRECT);
double speedInput, speedOutput, speedTarget;
PID speedPID(&speedInput, &speedOutput, &speedTarget,2,5,1, DIRECT);

void VisionBase::doLoop()
{
  rightMotor.doLoop();
  
  leftEncoder.updatePosition();
  rightEncoder.updatePosition();
  
  double left = leftEncoder.getPosition();
  double right = rightEncoder.getPosition();
  
}

void VisionBase::stopNow()
{    
  rightMotor.stopNow();
}

float VisionBase::getDistanceMadeSoFar()
{    
  return rightMotor.getDistanceMadeSoFar();
}

float VisionBase::encoderValue(float value)
{
  return (value * PI * wheelDiameter) / encoderResolution;
}



void VisionBase::update()
{

}
