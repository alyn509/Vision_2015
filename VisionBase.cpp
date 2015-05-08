#include "VisionBase.h"

double leftLast, rightLast;

// TODO: adjust the coefficients
double turnErrorInput, turnOutput, turnErrorTarget;
PID turnPID(&turnErrorInput, &turnOutput, &turnErrorTarget,2,5,1, DIRECT);
double speedInput, speedOutput, speedTarget;
PID speedPID(&speedInput, &speedOutput, &speedTarget,0.2,0.1,1, DIRECT);

void VisionBase::init()
{
  frontSensor.initPin(frontSensorPin);
  
  leftMotor.init();
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
  
  turnPID.SetSampleTime(9);
  speedPID.SetSampleTime(9);
  speedPID.SetOutputLimits(-10000.0, 10000.0);
  speedPID.SetMode(AUTOMATIC);
  turnPID.SetMode(AUTOMATIC);
  
  leftLast = 0;
  rightLast = 0;
  speedTarget = 0;
  turnErrorTarget = 0;
  
  state = 0;
}

void VisionBase::moveForward(float distance)
{
  leftMotor.setDirectionForward();
  rightMotor.setDirectionForward();
  leftEncoder.reset();
  rightEncoder.reset();
  leftLast = 0;
  rightLast = 0;
  speedTarget = distanceToEncoderTicks(distance);
}

void VisionBase::moveBackward(float distance)
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

void VisionBase::doLoop()
{
  leftMotor.doLoop();
  rightMotor.doLoop();
  
  leftEncoder.updatePosition();
  rightEncoder.updatePosition();
  
  switch (state) {
    case 0:
      moveForward(-10);
      state = STATE_STOP;
      break;
    default:
      state.doLoop();
  }
}

void VisionBase::stopNow()
{
  leftMotor.stopNow();
  rightMotor.stopNow();
}

double VisionBase::encoderValue(double value)
{
  return (value * PI * wheelDiameter) / encoderResolution;
}

double VisionBase::distanceToEncoderTicks(double distance)
{
  return (distance * encoderResolution) / (PI * wheelDiameter);
}

void VisionBase::update()
{
   double left = leftEncoder.getPosition();
   double right = rightEncoder.getPosition();
   
   turnErrorInput = left - right;
   speedInput = (left + right) / 2;
   speedPID.Compute();
   turnPID.SetOutputLimits(-abs(speedOutput), abs(speedOutput));
   turnPID.Compute();
   leftMotor.setSpeed(speedOutput - turnOutput);
   rightMotor.setSpeed(speedOutput + turnOutput);
   leftLast = left;
   rightLast = right;
   
   Serial.print("L ");
   Serial.print(left);
   Serial.print(" R ");
   Serial.print(right);
   Serial.print(" ST ");
   Serial.print(speedTarget);
   Serial.print(" SO ");
   Serial.print(speedOutput);
   Serial.print(" TO ");
   Serial.println(turnOutput);
}
