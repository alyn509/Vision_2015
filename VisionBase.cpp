#include "VisionBase.h"

void VisionBase::init()
{
  frontLeft.initPin(frontLeftSensorPin);
  //frontFront.initPin(frontFrontSensorPin);
  frontRight.initPin(frontRightSensorPin);
  
  backLeft.initPin(backLeftSensorPin);
  backRight.initPin(backRightSensorPin);
  backLow.initPin(backLowSensorPin);

  
  leftMotor.initDirectionForward(HIGH);
  leftMotor.initPins(rightMotorEnablePin, leftMotorDirectionPin, rightMotorStepPin);
  
  rightMotor.init();
  rightMotor.initDirectionForward(LOW);
  rightMotor.initPins(rightMotorEnablePin, rightMotorDirectionPin, rightMotorStepPin);
  rightMotor.initDelays(defaultStartSpeedDelay, highPhaseDelay, pauseSpeedDelay, delayBeforeTurnOff);
  rightMotor.initSizes(wheelDiameter, wheelRevolutionSteps,distanceBetweenWheels);  
  
  sensorScanner.attach(sensorScannerPin);
  sensorScanner.write(sensorScannerMiddle);
  
  directionMovement = NONE;
  obstructionDetected = false;
  ignoredSensors = false;
  pinMode(colorRedPin, INPUT);
  oppositeSide = (digitalRead(colorRedPin) == HIGH);// = false;
}

void VisionBase::setTacticDelays(int tactic)
{
  rightMotor.setTacticDelays(tactic);
}

void VisionBase::setStartDelays(unsigned long startDelay)
{
  rightMotor.initDelays(startDelay, highPhaseDelay, pauseSpeedDelay, delayBeforeTurnOff);
}

void VisionBase::moveForward(float distance, unsigned long step_delay)
{       
  directionMovement = FRONT;       
  rightMotor.setTargetDelay(step_delay);
  leftMotor.setDirectionForward();
  rightMotor.setDirectionForward();
  rightMotor.doDistanceInCm(distance);
}

void VisionBase::moveBackward(float distance, unsigned long step_delay)
{    
  directionMovement = BACK;   
  rightMotor.setTargetDelay(step_delay);
  leftMotor.setDirectionBackward();
  rightMotor.setDirectionBackward();  
  rightMotor.doDistanceInCm(distance);
}

void VisionBase::turnLeft(int angle)
{
  directionMovement = LEFT; 
  rightMotor.setTargetDelay(5000);
  leftMotor.setDirectionBackward();
  rightMotor.setDirectionForward();
  rightMotor.doRotationInAngle(angle); 
}

void VisionBase::turnRight(int angle)
{  
  directionMovement = RIGHT;    
  rightMotor.setTargetDelay(5000);
  leftMotor.setDirectionForward();
  rightMotor.setDirectionBackward();
  rightMotor.doRotationInAngle(angle);
}

void VisionBase::setSpecial()
{
  rightMotor.setSpecial();
}

void VisionBase::resetSpecial()
{
  rightMotor.resetSpecial();
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
  return frontLeft.detect() || frontRight.detect();
}

boolean VisionBase::backDetected()
{
  return backLeft.detect() || backRight.detect() || backLow.detect();
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
  if (frontDetected() && !ignoredSensors && directionMovement == FRONT)
  {
    obstructionDetected = true;
  }
  if (backDetected() && !ignoredSensors && directionMovement == BACK)
    obstructionDetected = true;
}

void VisionBase::doLoop()
{
  rightMotor.doLoop();
  if (sensorToggleTimer > sensorScannerToggleInterval)
  {
    if (sensorScanner.read() == sensorScannerLeft)
      sensorScanner.write(sensorScannerRight);
    else
      sensorScanner.write(sensorScannerLeft);
    sensorToggleTimer = 0;
  }
}

void VisionBase::stopNow()
{    
  rightMotor.stopNow();
}

float VisionBase::getDistanceMadeSoFar()
{    
  return rightMotor.getDistanceMadeSoFar();
}
