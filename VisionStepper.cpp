#include "VisionStepper.h"

// stepState states
#define STEP_LOW 0
#define STEP_HIGH 1
#define STOPPING 2
#define STOP 3

void VisionStepper::init()
{
  stepState = STATE_STOP;
  highPhaseDelay = 20;
  forwardDirection = HIGH;
}

void VisionStepper::initDirectionForward(boolean forward)
{
  forwardDirection = forward;
}

void VisionStepper::initPins(int enablePin, int directionPin, int stepPin)
{
  this->enablePin = enablePin;
  this->directionPin = directionPin;
  this->stepPin = stepPin;
  
  pinMode(directionPin, OUTPUT);
  directionPinState = forwardDirection;
  digitalWrite(directionPin, directionPinState);
  
  pinMode(enablePin, OUTPUT);
  enablePinState = LOW;
  digitalWrite(enablePin, enablePinState);
  
  pinMode(stepPin, OUTPUT);
  stepPinState = LOW;
  digitalWrite(stepPin, stepPinState);
}

void VisionStepper::initSizes(float wheelDiameter, int wheelRevolutionSteps, float distanceBetweenWheels)
{ 
  float wheelCircumference = wheelDiameter * PI;
  stepCmRatio = (wheelRevolutionSteps / wheelCircumference) * 2;
  float bigCircumference = PI * distanceBetweenWheels; //106.76 ;  3.14 * distanceBetweenWheels
  float degreeCmRatio = bigCircumference/360; // 0.2965;  bigCircumference/360
  degreeStepRatio = degreeCmRatio * stepCmRatio; //1.82;  degreeCmRatio * stepCmRatio
}

void VisionStepper::initStepCmRatio(float stepCmRatio)
{
  this->stepCmRatio = stepCmRatio;
}

void VisionStepper::setSpeed(double speed)
{
  if (speed == 0)
  {
    stopNow();
    return;
  }
  start();
  if (speed < 0)
  {
    speed = -speed;
    setDirectionBackward();
  }
  else
    setDirectionForward();
  currentDelay = 1000000.0 / speed;
  /*
  Serial.print(" CD ");
  Serial.print(currentDelay);
  Serial.print(" SPD ");
  Serial.print(speed);*/
}

void VisionStepper::doLoop()
{
  switch (stepState) {
    case STEP_LOW:
      stepPinState = LOW;
      digitalWrite(stepPin, stepPinState);
      stepState.waitMicros(currentDelay - highPhaseDelay, STEP_HIGH);
      break;
    case STEP_HIGH:
      stepPinState = HIGH;
      digitalWrite(stepPin, stepPinState);
      stepState.waitMicros(highPhaseDelay, STEP_LOW);
      break;
    case STOPPING:
      stepState.wait(20, STOP);
      break;
    case STOP:
      enablePinState = LOW;
      digitalWrite(enablePin, enablePinState);
      stepState = STATE_STOP;
      break;
    default:
      stepState.doLoop();
  }
}

void VisionStepper::pause()
{
  stopNow();
}

void VisionStepper::start()
{
  if (isOff()) {
    enablePinState = LOW;
    digitalWrite(enablePin, enablePinState);
    stepState = STEP_LOW;
  }
}

void VisionStepper::unpause()
{
  start();
}

void VisionStepper::stopNow()
{
  if (!isOff())
    stepState = STOPPING;
}

boolean VisionStepper::isOff()
{
  return !((stepState == STEP_LOW) || (stepState == STEP_HIGH));
}

boolean VisionStepper::isPaused()
{
  return isOff();
}

void VisionStepper::setDirectionForward()
{
  directionPinState = forwardDirection;
  digitalWrite(directionPin, directionPinState);
}

void VisionStepper::setDirectionBackward()
{
  directionPinState = !forwardDirection;
  digitalWrite(directionPin, directionPinState);
}

void VisionStepper::toggleDirection()
{
  directionPinState = !directionPinState;
  digitalWrite(directionPin, directionPinState);
}


