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
    stepState = STOP;
    return;
  }
  if (speed < 0)
  {
    speed = -speed;
    setDirectionBackward();
  }
  else
    setDirectionForward();
  currentDelay = 1.0 / speed;
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
  stopNow();
}

void VisionStepper::unpause()
{
  stepState = STEP_LOW;
}

void VisionStepper::stopNow()
{
  stepState = STATE_STOP;
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
  return stepState == STATE_STOP;
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


