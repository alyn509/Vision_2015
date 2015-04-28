#include "VisionState.h"

void VisionState::wait(unsigned long timeInMs, int nextState)
{
  originalState = *this;
  if (nextState == STATE_NEXT)
    stateToSetAfterWait = *this + 1;
  else if (nextState == STATE_LAST)
    stateToSetAfterWait = *this - 1;
  else
    stateToSetAfterWait = nextState;
  *this = STATE_WAIT;
  time = 0;
  timeToWait = timeInMs;
}

void VisionState::waitMicros(unsigned long timeInMicros, int nextState)
{
  originalState = *this;
  if (nextState == STATE_NEXT)
    stateToSetAfterWait = *this + 1;
  else if (nextState == STATE_LAST)
    stateToSetAfterWait = *this - 1;
  else
    stateToSetAfterWait = nextState;
  *this = STATE_WAIT_MICROS;
  timeMicros = 0;
  timeToWaitInMicros = timeInMicros;
}

void VisionState::waitFor(boolean (*functionToTestFor)(), int nextState)
{
  originalState = *this;
  if (nextState == STATE_NEXT)
    stateToSetAfterWait = *this + 1;
  else if (nextState == STATE_LAST)
    stateToSetAfterWait = *this - 1;
  else stateToSetAfterWait = nextState;
  *this = STATE_WAIT_FOR;
  testFunction = functionToTestFor;
}

void VisionState::doLoop()
{ 
  switch (state)
  {
    case STATE_STOP:
      break;
    case STATE_WAIT:
      if (time > timeToWait)
        *this = stateToSetAfterWait;
      break;
    case STATE_WAIT_MICROS:
      if (timeMicros > timeToWaitInMicros)
        *this = stateToSetAfterWait;
      break;
    case STATE_WAIT_FOR:
      if (testFunction())
        *this = stateToSetAfterWait;
      break;
  }
}

VisionState::operator int()
{
  return state;
}

int VisionState::operator =(const int state)
{
  if (state == STATE_NEXT)
    return ++(this->state);
  return (this->state = state);
}

VisionState& VisionState::operator+=(const int val) {
  *this = *this + val;
}

VisionState& VisionState::operator++() {
  *this += 1;
  return *this;
}

VisionState VisionState::operator++(int) {
  VisionState tmp(*this); // copy
  operator++(); // pre-incrementoriginalState = *this;
  return tmp;   // return old value
}
