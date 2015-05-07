#include "VisionBase.h"

void VisionBase::init()
{
  frontSensor.initPin(frontLeftSensorPin);
    
  leftMotor.initDirectionForward(HIGH);
  leftMotor.initPins(MotorEnablePin, leftMotorDirectionPin, MotorStepPin);
  
  rightMotor.init();
  rightMotor.initDirectionForward(LOW);
  rightMotor.initPins(MotorEnablePin, rightMotorDirectionPin, MotorStepPin);
  rightMotor.initDelays(defaultStartSpeedDelay, highPhaseDelay, pauseSpeedDelay, delayBeforeTurnOff);
  rightMotor.initSizes(wheelDiameter, wheelRevolutionSteps,distanceBetweenWheels);    
  
  leftEncoder.init(leftEncoderStepPin);
  rightEncoder.init(rightEncoderStepPin);
    
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
  lastPositionLeft = leftEncoder.getPosition();
  lastPositionRight = rightEncoder.getPosition();
}

void VisionBase::moveBackward(float distance, unsigned long step_delay)
{    
  directionMovement = BACK;   
  rightMotor.setTargetDelay(step_delay);
  leftMotor.setDirectionBackward();
  rightMotor.setDirectionBackward();  
  rightMotor.doDistanceInCm(distance);
  lastPositionLeft = leftEncoder.getPosition();
  lastPositionRight = rightEncoder.getPosition();
}

void VisionBase::turnLeft(int angle)
{
  directionMovement = LEFT; 
  rightMotor.setTargetDelay(5000);
  leftMotor.setDirectionBackward();
  rightMotor.setDirectionForward();
  rightMotor.doRotationInAngle(angle); 
  lastPositionLeft = leftEncoder.getPosition();
  lastPositionRight = rightEncoder.getPosition();
}

void VisionBase::turnRight(int angle)
{  
  directionMovement = RIGHT;
  rightMotor.setTargetDelay(5000);
  leftMotor.setDirectionForward();
  rightMotor.setDirectionBackward();
  rightMotor.doRotationInAngle(angle);
  lastPositionLeft = leftEncoder.getPosition();
  lastPositionRight = rightEncoder.getPosition();
}

bool VisionBase::leftMotorDir()
{
  return (directionMovement / 2) % 2;
}

bool VisionBase::rightMotorDir()
{  
  return directionMovement % 2;
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
  if (frontDetected() && !ignoredSensors && directionMovement == FRONT)
    obstructionDetected = true;
}

void VisionBase::doLoop()
{
  rightMotor.doLoop();
  
  leftEncoder.updatePosition();
  rightEncoder.updatePosition();
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
    int difference = leftEncoder.getPosition() - rightEncoder.getPosition();
    int deriv = difference - last;
    last = difference;
    integral += difference;
    int turn = 0.5 * difference + 0.1 * integral + 25.0 * deriv;
    if (turn > 30) turn = 30;
    if (turn < -30) turn = -30;
    leftMotor.doDistanceInCm(80 + turn);
    rightMotor.doDistanceInCm(80 - turn);
    
    /*Serial.print("dif: ");
    Serial.print(difference);
    Serial.print(" int: ");
    Serial.print(integral);
    Serial.print(" deriv: ");
    Serial.print(deriv);
    Serial.print(" turn: ");
    Serial.println(turn);
  }*/
}
