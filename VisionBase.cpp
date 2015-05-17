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
    
  attachServoz();

  directionMovement = NONE;
  ignoredSensors = false;
  
  if(sideButton.detect())
    state = YELLOW_SIDE;
  else
  {
    state = GREEN_SIDE;    
    pinMode(greenLed, OUTPUT);
    digitalWrite(greenLed, HIGH);
  }
}
void VisionBase::attachServoz()
{
  claw.attach(clawServoPin);                //closeRightClaw();
    
  leftPopcornHolder.attach(leftPopCornHolderPin);     //leftPopcornHolder.write(180);
  rightPopcornHolder.attach(rightPopCornHolderPin);   //rightPopcornHolder.write(0);
  
  leftArm.attach(leftArmServoPin);                    //closeLeftArm();
  rightArm.attach(rightArmServoPin);                  //closeRightArm();
  
  door.attach(doorServoPin);
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
  doDistanceInCM(distance, nextState);
}
void VisionBase::moveBackward(int distance,int pwmv, int nextState)
{  
  if(!newMovement || isResuming)
  {  
    directionMovement = FRONT;
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
  doDistanceInCM(distance, nextState);
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
  doAngleRotation(angle, nextState);
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
  doAngleRotation(angle, nextState);
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
      if(nextState == STATE_NEXT)
        state++;
      else
        state = nextState;
    }
  }
}
void VisionBase::doAngleRotation(int dist, int nextState)
{
  float steps = cmToSteps(2 * angleToSteps(dist));
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
      if(nextState == STATE_NEXT)
        state++;
      else
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
  if(directionMovement == FRONT)
  {
    int threshold = pwmValue - 10;
    int difference = leftEncoder.getPosition() - rightEncoder.getPosition();
    int deriv = difference - last;
    last = difference;
    integral += difference;
    int turn = 2.0 * difference + 0.0 * integral + 0.0 * deriv;
    if (turn > threshold) turn = threshold;
    if (turn < -threshold) turn = -threshold;
    if (leftMotor.isOn)
      leftMotor.moveForward(pwmValue - turn);
    if (rightMotor.isOn)
      rightMotor.moveForward(pwmValue + turn);
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
    case 0: 
      moveForward(30, 50,1);
      break; 
    case 1: 
      turnLeft(90, 50,2);
      break;
    case 2:
      break; 
    
/*    case 0: 
      moveForward(45, 50, STATE_NEXT);
      break;
    case 1:
      turnRight(90, 50, STATE_NEXT);
      break;
    case 2: 
      openRightArm();
      moveForward(45, 50, STATE_NEXT);
      break;
    case 3:
      turnRight(90, 50, STATE_NEXT);
      break;
    case 4: 
      moveForward(15, 50, STATE_NEXT);
      break;
    case 5:
      turnLeft(90, 50,STATE_NEXT);
      break;
    case 6: 
      moveForward(20, 50,STATE_NEXT);
      break;
    case 7:
      grabRightArm();
      state.wait(300,STATE_NEXT);
      break;
    case 8: 
      moveForward(7, 50,STATE_NEXT);
      break;
    case 9:
      turnRight(90, 50,STATE_NEXT);
      break;
    case 10: 
     // openLeftClaw();
   //   openRightClaw();
      moveForward(21, 50,STATE_NEXT);
      break;*/
  /*  case 11: 
      grabLeftClaw();
      grabRightClaw();
      state.wait(500,STATE_NEXT);
      break;  
    case 12:
      riseLift();
      state.wait(700,STATE_NEXT);
      break;*/
    case 11:
      turnRight(30, 50,STATE_NEXT);
      break;
    case 12: 
      moveBackward(12, 50,STATE_NEXT);
      break;
    case 13:
      turnLeft(30, 50,STATE_NEXT);
      break;
    case 14: 
      moveForward(17, 50,STATE_NEXT);
      break;
    case 15: 
      openLeftArm();
      state.wait(100,STATE_NEXT);
      break;
    case 16: 
      moveBackward(30, 50,STATE_NEXT);
      break;
    case 17: 
      closeLeftArm();
      state.wait(100,STATE_NEXT);
      break;
    case 18: 
      moveBackward(30, 50,STATE_NEXT);
      break;
    case 19: 
      openLeftArm();
      state.wait(100,STATE_NEXT);
      break;
    case 20: 
      moveBackward(30, 50,STATE_STOP);
      break;
    case STATE_STOP:
      break;
    default:
      state.doLoop();   
  }
    
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
  claw.write(150);
}

void VisionBase::closeClaw()
{
  claw.write(12);
}

void VisionBase::grabClaw()
{
  claw.write(40);
}
    
void VisionBase::openDoow()    // unimplemented
{
  door.write(180);
}

void VisionBase::closeDoor()    // unimplemented
{
  door.write(0);
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
    
void VisionBase::riseLift()
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
  }
}

void VisionBase::lowerLift()
{
  digitalWrite(downLiftPin, HIGH);
}
void VisionBase::stopLift()
{
  digitalWrite(upLiftPin, LOW);
  digitalWrite(downLiftPin, LOW);
}
