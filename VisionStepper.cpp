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
  
  special = false;
}

void VisionStepper::initDelays(unsigned long startSpeedDelay, unsigned long highPhaseDelay, unsigned long pauseSpeedDelay, unsigned long delayBeforeTurnOff)
{
  this->startSpeedDelay = startSpeedDelay;
  this->highPhaseDelay = highPhaseDelay;
  this->pauseSpeedDelay = pauseSpeedDelay;
  this->delayBeforeTurnOff = delayBeforeTurnOff;
  currentDelay = startSpeedDelay;
}

void VisionStepper::setTacticDelays(int tactic)
{
  switch(tactic)
  {
    case CLASSIC_START:  
      stepSpeedCounterAcceleration = 8;
      stepSpeedCounterSlowing = 6;
      break;
    case AGGRESSIVE_START:  
      stepSpeedCounterAcceleration = 30; 
      stepSpeedCounterSlowing = 20;  
      break;
    case SLOW_START:
      stepSpeedCounterAcceleration = 2;
      stepSpeedCounterSlowing = 1;
      break;      
  }
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
  switch (motorState)
  {
    case STARTING:
      motorState = RUNNING;
      enableState = TURN_ON;
      break;
    case RUNNING:
      if (stepsRemaining <= stepSpeedCounter / stepSpeedCounterSlowing)
        motorState = STOPPING_SLOWING;
      break;
    case PAUSING_SLOWING:
      savedWhenPausingDelay = targetDelay;
      savedDeacceleration = stepSpeedCounterSlowing;
      stepSpeedCounterSlowing = 50;
      setTargetDelay(pauseSpeedDelay);
      motorState = PAUSING;
      break;
    case PAUSING:
      if (speedState == CONSTANT)
      {
        stepSpeedCounterSlowing = savedDeacceleration;
        enableState = DELAYED_TURN_OFF;
        motorState = PAUSED;
      }
      break;
    case PAUSED:
      break;
    case RESUME:
      setTargetDelay(savedWhenPausingDelay);
      if (enableState != ON)
        enableState = TURN_ON;
      motorState = RUNNING;
      break;
    case STOPPING_SLOWING:
      setTargetDelay(startSpeedDelay);
      motorState = STOPPING;
      break;
    case STOPPING:
      if (stepsRemaining == 0)
      {
        enableState = DELAYED_TURN_OFF;
        motorState = STOPPED;
      }
      break;
    case STOPPED:
      break;
    default:
      motorState.doLoop();
  }
  switch (enableState)
  {
    case TURN_ON:
      speedState = START;
      enablePinState = HIGH;
      digitalWrite(enablePin, enablePinState);
      enableState = ON;
      break;
    case ON:
      break;
    case DELAYED_TURN_OFF:
      speedState = STOP;
      enablePinState = LOW;
      enableState.wait(delayBeforeTurnOff, TURN_OFF);
      break;
    case TURN_OFF:
      digitalWrite(enablePin, enablePinState);
      enableState = OFF;
      break;
    case OFF:
      break;
    default:
      enableState.doLoop();
  }
  switch (speedState)
  {
    case ACCELERATING:
      if (currentDelay <= targetDelay)
        speedState = CONSTANT;
      break;
    case SLOWING:
      if (currentDelay >= targetDelay)
        speedState = CONSTANT;
      break;
    case CONSTANT:
      break;
    case UNDETERMINED:
      if (currentDelay < targetDelay)
        speedState = SLOWING;
      else if (currentDelay > targetDelay)
        speedState = ACCELERATING;
      else
        speedState = CONSTANT;
      break;
    case START:
      stepSpeedCounter = 0;
      speedState = ACCELERATING;
      stepState = STEP_LOW;
      break;
    case STOP:
      stepState = STATE_STOP;
      break;
    default:
      speedState.doLoop();
  }
  switch (stepState) {
    case STEP_LOW:
      stepsMadeSoFar++;
      stepsRemaining--;
      if (speedState == ACCELERATING)
        stepSpeedCounter += stepSpeedCounterAcceleration;
      else if (speedState == SLOWING)
      {
        stepSpeedCounter -= stepSpeedCounterSlowing;
        if (stepSpeedCounter < 0)
            stepSpeedCounter = 0;
      }
      currentDelay = startSpeedDelay / sqrt(stepSpeedCounter + 1);
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
float VisionStepper::computeSpeed()
{
  return startSpeedDelay * 10 / sqrt(1 * stepSpeedCounter + 100);
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
  motorState = STARTING;
}

void VisionStepper::doDistanceInCm(float distance)
{
  doSteps(distance * stepCmRatio);
}

void VisionStepper::doRotationInAngle(float angle)
{
  doSteps(angle * degreeStepRatio);
}

void VisionStepper::setSpecial()
{
  special = true;
}
void VisionStepper::resetSpecial()
{
  special = false;
}


