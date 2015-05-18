#include "VisionBase.h"

void VisionBase::init()
{  
  sideButton.initPin(startButtonPin);
  sideButton.setAsPullup();
  
  frontLeftSensor.initPin(frontLeftSensorPin);
  frontRightSensor.initPin(frontRightSensorPin);
  
  liftLimitatorSensor.initPin(liftLimitatorSensorPin);
  liftLimitatorSensor.setAsPullup();
   
  leftMotor.init(leftMotorFw, leftMotorBw);
  rightMotor.init(rightMotorFw, rightMotorBw);
  
  leftEncoder.init(leftEncoderStepPin);
  rightEncoder.init(rightEncoderStepPin); 
    
  pinMode(popCornGrabberDCPin, OUTPUT);
  pinMode(upLiftPin, OUTPUT);
  pinMode(downLiftPin, OUTPUT);
    
  state = 0;
  deviceState = 0;
  attachServoz();

  directionMovement = NONE;  
  ignoredSensors = false;
  /*
  if(sideButton.detect())
    state = YELLOW_SIDE;
  else
  {
    state = GREEN_SIDE;    
    pinMode(greenLed, OUTPUT);
    digitalWrite(greenLed, HIGH);
  }*/
}
void VisionBase::attachServoz()
{
  claw.attach(clawServoPin);                //closeRightClaw();
    
  leftPopcornHolder.attach(leftPopCornHolderPin);     //leftPopcornHolder.write(180);
  rightPopcornHolder.attach(rightPopCornHolderPin);   //rightPopcornHolder.write(0);
  
  leftArm.attach(leftArmServoPin);                    //closeLeftArm();
  rightArm.attach(rightArmServoPin);                  //closeRightArm();
  
  door.attach(doorServoPin);    closeDoor();
}

void VisionBase::moveForward(int distance, int pwmv, int nextState)
{  
  if(!newMovement || isResuming)
  {  
    directionMovement = FRONT;
    leftMotor.moveForward(pwmv);
    rightMotor.moveForward(pwmv);
    if(!isResuming)
    {
      leftEncoder.resetPosition();
      rightEncoder.resetPosition();
    }
    isResuming = false;
    pwmValue = pwmv;
  }
  doDistanceInCM(distance, getState(nextState, state));
}
void VisionBase::moveBackward(int distance,int pwmv, int nextState)
{  
  if(!newMovement || isResuming)
  {  
    directionMovement = BACK;
    leftMotor.moveBackward(pwmv);
    rightMotor.moveBackward(pwmv);
    if(!isResuming)
    {
      leftEncoder.resetPosition();
      rightEncoder.resetPosition();
    }
    isResuming = false;
    pwmValue = pwmv;
  }
  doDistanceInCM(distance, getState(nextState, state));
}
void VisionBase::turnLeft(int angle,int pwmv, int nextState)
{  
  if(!newMovement || isResuming)
  {  
    directionMovement = LEFT;
    leftMotor.moveBackward(pwmv);
    rightMotor.moveForward(pwmv);
    if(!isResuming)
    {
      leftEncoder.resetPosition();
      rightEncoder.resetPosition();
    }
    isResuming = false;
    pwmValue = pwmv;
  }
  doAngleRotation(angle, getState(nextState, state));
}

void VisionBase::turnRight(int angle,int pwmv, int nextState)
{    
  if(!newMovement || isResuming)
  {  
    directionMovement = RIGHT;
    leftMotor.moveForward(pwmv);
    rightMotor.moveBackward(pwmv);
    if(!isResuming)
    {
      leftEncoder.resetPosition();
      rightEncoder.resetPosition();
    }
    isResuming = false;
    pwmValue = pwmv;
  }  
  doAngleRotation(angle, getState(nextState, state));
}
int VisionBase::getState(int stateToGet, int originalState)
{
  if(stateToGet == STATE_NEXT)
    return originalState + 1;
  else
    return stateToGet;
}
float VisionBase::cmToSteps(float value)
{
  return (1.0 * value * encoderResolution) / (PI * wheelDiameter);
}
float VisionBase::angleToSteps(float value)
{
  return (1.0 * value * distanceBetweenWheels * PI) / 360.0;
}
void VisionBase::doDistanceInCM(int dist, int nextState)
{
  float steps = cmToSteps(dist);
  if(!newMovement)
  {
    newMovement = true;
    lastPositionLeft = leftEncoder.getPosition();
    lastPositionRight = rightEncoder.getPosition();
  }
  else
  {
    if(leftEncoder.getPosition() - lastPositionLeft >= steps)
      leftMotor.stopMotor();
    if(rightEncoder.getPosition() - lastPositionRight >= steps)
      rightMotor.stopMotor();
    if(!leftMotor.isOn && !rightMotor.isOn && !isPaused)
    {
      newMovement = false;
      state = nextState;
    }
  }
}
void VisionBase::doAngleRotation(int dist, int nextState)
{
  float steps = cmToSteps(angleToSteps(dist));
  if(!newMovement)
  {
    newMovement = true;
    lastPositionLeft = leftEncoder.getPosition();
    lastPositionRight = rightEncoder.getPosition();
  }
  else
  {
    if(leftEncoder.getPosition() - lastPositionLeft >= steps)
      leftMotor.stopMotor();
    if(rightEncoder.getPosition() - lastPositionRight >= steps)
      rightMotor.stopMotor();
    if(!leftMotor.isOn && !rightMotor.isOn && !isPaused)
    {
      newMovement = false;
      state = nextState;
    }
  }
}

void VisionBase::pause()
{ 
  isPaused = true;
  stateBeforePause = state;
  state = PAUSED;
  rightMotor.stopMotor();      
  leftMotor.stopMotor(); 
}
void VisionBase::unpause()
{
  state = stateBeforePause;
  isPaused = false;
  isResuming = true;
}

int integral, last;
void VisionBase::update()
{
  /*
  Serial.print(" L: ");
  Serial.print(leftEncoder.getPosition());
  Serial.print(" R: ");
  Serial.println(rightEncoder.getPosition());
 */
  int threshold = pwmValue - 10;
  int difference = leftEncoder.getPosition() - rightEncoder.getPosition();
  int deriv = difference - last;
  last = difference;
  integral += difference;
  int turn = 2.0 * difference + 0.0 * integral + 0.0 * deriv;
  if (turn > threshold) turn = threshold;
  if (turn < -threshold) turn = -threshold;
  
  if(directionMovement == FRONT)
  {
    if (leftMotor.isOn)
      leftMotor.moveForward(pwmValue - turn);
    if (rightMotor.isOn)
      rightMotor.moveForward(pwmValue + turn);
  }
  if(directionMovement == BACK)
  {
    if (leftMotor.isOn)
      leftMotor.moveBackward(pwmValue - turn);
    if (rightMotor.isOn)
      rightMotor.moveBackward(pwmValue + turn);
  }
  if(directionMovement == LEFT)
  {
    if (leftMotor.isOn)
      leftMotor.moveBackward(pwmValue - turn);
    if (rightMotor.isOn)
      rightMotor.moveForward(pwmValue + turn);
  }
  if(directionMovement == RIGHT)
  {
    if (leftMotor.isOn)
      leftMotor.moveForward(pwmValue - turn);
    if (rightMotor.isOn)
      rightMotor.moveBackward(pwmValue + turn);
  }
}

bool VisionBase::frontDetected()
{
  return frontLeftSensor.detect() || frontRightSensor.detect();
}

void VisionBase::doLoop()
{   
  switch(state)
  {
    case 0:  moveForward(60, 30, STATE_NEXT);break;
    case 1:  turnLeft(90, 30, STATE_NEXT);break;
    case 2:  moveForward(93, 30, STATE_NEXT);break;
    case 3:  deviceState = 10; state.wait(500, STATE_NEXT);break;
    case 4:  moveBackward(93, 30, STATE_NEXT);break;
    case 5:  turnLeft(190, 30, STATE_NEXT);break;
    case 6:  moveForward(40, 30, STATE_NEXT);break;
    case 7:  deviceState = 21; state.wait(1000, STATE_NEXT);break;
    case 8:  turnLeft(80, 30, STATE_NEXT);break;
    case 9:  moveForward(50, 30, STATE_NEXT);break;
    case 10:  deviceState = 21; state.wait(1000, STATE_NEXT);break;
    case 11:  turnRight(120, 30, STATE_NEXT);break;
    case 12:  moveForward(48, 30, STATE_NEXT);break;
    case 13:  deviceState = 15; state.wait(1000, STATE_NEXT);break;
    case 14:  turnLeft(90, 30, STATE_NEXT);break;
    case 15:  unlockDoor(); moveForward(10, 30, STATE_NEXT);break;
    case 16: openClaw(); state.wait(100, STATE_NEXT);break;
    case 17: openDoor(); state.wait(100, STATE_NEXT);break;
    case 18: moveBackward(25, 30, STATE_NEXT);break;
    case 19: deviceState = 30; turnRight(150, 30, STATE_NEXT);break;
    case 20: closeDoor(); moveForward(70, 30, STATE_NEXT);break;
    case 21: turnRight(35, 30, STATE_NEXT);break;
    case 22: moveBackward(8, 30, STATE_NEXT);break;
    case 23: turnLeft(38, 30, STATE_NEXT);break;
    case 24: moveForward(25, 30, STATE_NEXT);break;
    case 25: closeClaw();
             state.wait(200, STATE_NEXT);break;
    case 26: openLeftArm();
             state.wait(200, STATE_NEXT);break;
    case 27: moveBackward(30, 30, STATE_NEXT);break;
    case 28: closeLeftArm();
             state.wait(100, STATE_NEXT);break;
    case 29: moveBackward(30, 30, STATE_NEXT);break;
    case 30: openLeftArm();
             state.wait(100, STATE_NEXT);break;
    case 31: moveBackward(20, 30, STATE_NEXT);break;
    case 32: turnRight(60, 30, STATE_NEXT);break;
    case 33: moveForward(60, 30, STATE_NEXT);break;
    case 34: turnRight(30, 30, STATE_NEXT);break;
    case 35: moveForward(30, 30, STATE_NEXT);break;
    case 36: turnLeft(90, 30, STATE_NEXT);break;
    case 37: moveForward(30, 30, STATE_NEXT);break;
    case 38: openClaw(); moveBackward(30, 30, STATE_NEXT);break;
    /************************************************************************************************************************/
    
 
    case STATE_STOP: break;
    default: state.doLoop();   
  } 
  switch(deviceState)
  {
    case 0:  riseLift(STATE_NEXT); break;     
    case 1:  openClaw(); deviceState.wait(500, STATE_NEXT); break; 
    case 2:  lowerLift(STATE_NEXT); break; 
    case 3:  stopLift(STATE_STOP); break; 
    
    case 10:  closeClaw(); deviceState.wait(500, STATE_NEXT); break; 
    case 11:  riseLift(STATE_STOP); break;        
    
    case 15:  openClaw(); deviceState.wait(500, STATE_NEXT); break; 
    case 16:  lowerLift(STATE_NEXT); break; 
    case 17:  stopLift(deviceState); closeClaw(); deviceState.wait(500, STATE_STOP); break; 
   
    case 21:  openClaw(); deviceState.wait(500, STATE_NEXT); break; 
    case 22:  lowerLift(STATE_NEXT); break; 
    case 23:  stopLift(deviceState); closeClaw(); deviceState.wait(500, STATE_NEXT); break; 
    case 24:  riseLift(STATE_STOP); break;       
        
    case 30:  halfLowerLift(STATE_NEXT); break; 
    case 31:  stopLift(deviceState); deviceState.wait(500, STATE_STOP); break; 
    
    case STATE_STOP: break;   
    default: deviceState.doLoop();      
  }
  
  leftEncoder.updatePosition();
  rightEncoder.updatePosition();
    
  if(liftLimitatorSensor.detect() && goingUp == true)
  {
    digitalWrite(upLiftPin, LOW);  
    goingUp = false;
  }
}

void VisionBase::stopNow()
{      
  rightMotor.stopMotor();      
  leftMotor.stopMotor();  
  isStopped = true;
}
/********************************************************* Servoz *********************************************************/
void VisionBase::openLeftArm()
{
  leftArm.write(30);
}

void VisionBase::closeLeftArm()
{
  leftArm.write(110);
}

void VisionBase::grabLeftArm()
{
  leftArm.write(90);
}

void VisionBase::openRightArm()
{
  rightArm.write(100);
}

void VisionBase::closeRightArm()
{
  rightArm.write(20);
}

void VisionBase::grabRightArm()
{
  rightArm.write(40);
}
 
void VisionBase::openClaw()
{
  claw.write(100);
}

void VisionBase::closeClaw()
{
  claw.write(0);
}
    
void VisionBase::openDoor()   
{
  door.write(180);
}
void VisionBase::unlockDoor()   
{
  door.write(100);
}

void VisionBase::closeDoor()   
{
  door.write(64);
}
  
void VisionBase::releaseLeftPopcorn()
{
  leftPopcornHolder.write(130);
}

void VisionBase::releaseRightPopcorn()
{
  rightPopcornHolder.write(60);
}
    
void VisionBase::gatherPopcorn()
{
  digitalWrite(popCornGrabberDCPin, HIGH);
}

void VisionBase::stopGatherPopcorn()
{
  digitalWrite(popCornGrabberDCPin, LOW);
}
    
void VisionBase::riseLift(int stateNext)
{
  if(!liftLimitatorSensor.detect())
  {
    digitalWrite(upLiftPin, HIGH);
    goingUp = true;
  }
  else
  {
    digitalWrite(upLiftPin, LOW);
    goingUp = false;
    deviceState = getState(stateNext, deviceState);
  }
}

void VisionBase::lowerLift(int stateNext)
{
  digitalWrite(downLiftPin, HIGH);
  deviceState.wait(750, stateNext);
}
void VisionBase::halfLowerLift(int stateNext)
{
  digitalWrite(downLiftPin, HIGH);
  deviceState.wait(375, stateNext);
}
void VisionBase::stopLift(int stateNext)
{
  digitalWrite(upLiftPin, LOW);
  digitalWrite(downLiftPin, LOW);
  goingUp = false;
  deviceState = getState(stateNext, deviceState);
}
