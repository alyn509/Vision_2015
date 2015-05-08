#include "VisionBase.h"

double leftLast, rightLast;

// TODO: adjust the coefficients
double turnErrorInput, turnOutput, turnErrorTarget;
PID turnPID(&turnErrorInput, &turnOutput, &turnErrorTarget,2,5,1, DIRECT);
double speedInputL, speedInputR, speedOutputR, speedOutputL, speedTargetL, speedTargetR;
PID speedPIDL(&speedInputL, &speedOutputL, &speedTargetL,0.1,0.08,1, DIRECT);
PID speedPIDR(&speedInputR, &speedOutputR, &speedTargetR,0.1,0.08,1, DIRECT);

void VisionBase::init()
{
  frontSensor.initPin(frontSensorPin);
  
  leftMotor.init();
  leftMotor.initDirectionForward(HIGH);
  leftMotor.initPins(leftMotorEnablePin, leftMotorDirectionPin, leftMotorStepPin);
  leftMotor.initSizes(wheelDiameter, wheelRevolutionSteps,distanceBetweenWheels);
  
  rightMotor.init();
  rightMotor.initDirectionForward(HIGH);
  rightMotor.initPins(rightMotorEnablePin, rightMotorDirectionPin, rightMotorStepPin);
  rightMotor.initSizes(wheelDiameter, wheelRevolutionSteps,distanceBetweenWheels);
  
  leftEncoder.init(leftEncoderStepPin, leftEncoderBPin);
  rightEncoder.init(rightEncoderStepPin, rightEncoderBPin);
  
  obstructionDetected = false;
  ignoredSensors = false;
  
  turnPID.SetSampleTime(99);
  speedPIDL.SetSampleTime(99);
  speedPIDL.SetOutputLimits(-1000.0, 1000.0);
  speedPIDL.SetMode(AUTOMATIC);
  speedPIDR.SetSampleTime(99);
  speedPIDR.SetOutputLimits(-1000.0, 1000.0);
  speedPIDR.SetMode(AUTOMATIC);
  turnPID.SetMode(AUTOMATIC);
  
  leftLast = 0;
  rightLast = 0;
  speedTargetL = 0;
  speedTargetR = 0;
  turnErrorTarget = 0;
  
  state = 0;
}

void VisionBase::moveForward(double distance)
{
  move(distance);
  speedTargetL = distanceToEncoderTicks(distance);
  speedTargetR = distanceToEncoderTicks(distance);
}

void VisionBase::moveBackward(double distance)
{
  move(distance);
  speedTargetL = -distanceToEncoderTicks(distance);
  speedTargetR = -distanceToEncoderTicks(distance);
}

void VisionBase::turnLeft(double distance)
{
  move(distance);
  speedTargetL = -distanceToEncoderTicks(distance);
  speedTargetR = distanceToEncoderTicks(distance);
}

void VisionBase::turnRight(double distance)
{
  move(distance);
  speedTargetL = distanceToEncoderTicks(distance);
  speedTargetR = -distanceToEncoderTicks(distance);
}

void VisionBase::move(double distance)
{
  leftEncoder.reset();
  rightEncoder.reset();
  leftLast = 0;
  rightLast = 0;
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

boolean VisionBase::frontDetected()
{
  return frontSensor.detect();
}

boolean VisionBase::isStopped()
{
  return rightMotor.isOff() && leftMotor.isOff();
}

boolean VisionBase::isPaused()
{
  return rightMotor.isPaused() && leftMotor.isPaused();
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
      moveForward(30);
      leftMotor.setSpeed(1000);
      rightMotor.setSpeed(1000);
      state.wait(10000, STATE_STOP);
      break;
    case 1:
      turnLeft(10);
      state.wait(1000, STATE_NEXT);
      break;
    case 2:
      turnRight(10);
      state.wait(1000, STATE_NEXT);
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
   /*
   turnErrorInput = left - right;
   speedInputL = left;
   speedInputR = right;
   speedPIDL.Compute();
   speedPIDR.Compute();
   turnPID.SetOutputLimits(abs(speedOutputL)/2.0, abs(speedOutputL));
   turnPID.Compute();
   leftMotor.setSpeed(speedOutputL - turnOutput);
   rightMotor.setSpeed(speedOutputR + turnOutput);
   leftLast = left;
   rightLast = right;*/
   /*
   Serial.print("L ");
   Serial.print(left);
   Serial.print(" R ");
   Serial.print(right);
   Serial.print(" STL ");
   Serial.print(speedTargetL);
   Serial.print(" STR ");
   Serial.print(speedTargetR);
   Serial.print(" SOL ");
   Serial.print(speedOutputL);
   Serial.print(" SOR ");
   Serial.print(speedOutputR);
   Serial.print(" TO ");
   Serial.println(turnOutput);*/
}
