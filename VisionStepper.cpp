#include "VisionStepper.h"

// motorState states
#define STARTING 0
#define RUNNING 1
#define PAUSING_SLOWING 2
#define PAUSING 3
#define PAUSED 4
#define RESUME 5
#define STOPPING_SLOWING 6
#define STOPPING 7
#define STOPPED 8

// enableState states
#define TURN_ON 0
#define ON 1
#define DELAYED_TURN_OFF 2
#define TURN_OFF 3
#define OFF 4

// speedState states
#define ACCELERATING 0
#define SLOWING 1
#define CONSTANT 2
#define UNDETERMINED 3
#define START 4
#define STOP 5

// stepState states
#define STEP_LOW 0
#define STEP_HIGH 1

const unsigned long waitBeforeTurningOff = 500;

void VisionStepper::init()
{
  stepsMadeSoFar = 0;
  stepsRemaining = 0;
  motorState = STOPPED;
  speedState = STATE_STOP;
  enableState = OFF;
  stepState = STATE_STOP;
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

void VisionStepper::initDelays(unsigned long startSpeedDelay, unsigned long highPhaseDelay, unsigned long pauseSpeedDelay, unsigned long delayBeforeTurnOff)
{
  this->startSpeedDelay = startSpeedDelay;
  this->highPhaseDelay = highPhaseDelay;
  this->pauseSpeedDelay = pauseSpeedDelay;
  this->delayBeforeTurnOff = delayBeforeTurnOff;
  currentDelay = startSpeedDelay;
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

void VisionStepper::doLoop()
{
  switch (stepState) {
    case STEP_LOW:
      stepsMadeSoFar++;
      stepsRemaining--;
      stepPinState = LOW;
      digitalWrite(stepPin, stepPinState);
      stepState.waitMicros(currentDelay, STEP_HIGH);
      break;
    case STEP_HIGH:
      stepPinState = HIGH;
      digitalWrite(stepPin, stepPinState);
      stepState.waitMicros(highPhaseDelay, STEP_LOW);
      break;
    default:
      stepState.doLoop();
  }
}
float VisionStepper::getDistanceMadeSoFar()
{
  return stepsMadeSoFar / stepCmRatio;
}
float VisionStepper::getDistanceRemainedToDo()
{
  return stepsRemaining / stepCmRatio;
}

void VisionStepper::pause()
{
  if (motorState == RUNNING || motorState == STOPPING)
    motorState = PAUSING_SLOWING;
}

void VisionStepper::unpause()
{
  if (motorState == PAUSING_SLOWING || motorState == PAUSING || motorState == PAUSED)
    motorState = RESUME;
}

void VisionStepper::stopNow()
{
  motorState = STOPPING;
}

void VisionStepper::setTargetDelay(unsigned long targetDelay)
{
  if (this->targetDelay == targetDelay)
    return;
  this->targetDelay = targetDelay;
  speedState = UNDETERMINED;
}

boolean VisionStepper::isOff()
{
  return motorState == STOPPED;
}

boolean VisionStepper::isPaused()
{
  return ((motorState == PAUSING_SLOWING) || (motorState == PAUSING) || (motorState == PAUSED));
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

boolean VisionStepper::isAtTargetSpeed()
{
  return motorState == RUNNING;
}

void VisionStepper::doSteps(unsigned long stepNumber)
{
  stepsMadeSoFar = 0;
  stepsRemaining = stepNumber;
  if(motorState != RUNNING)
  {    
    motorState = STARTING;
  }
}

void VisionStepper::doDistanceInCm(float distance)
{
  doSteps(distance * stepCmRatio);
}

void VisionStepper::doRotationInAngle(float angle)
{
  doSteps(angle * degreeStepRatio);
}


