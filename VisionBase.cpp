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
  int state_next = 0;
  if(nextState == STATE_NEXT)
    state_next = state + 1;
  else
    state_next = nextState;
  doDistanceInCM(distance, state_next);
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
  int state_next = 0;
  if(nextState == STATE_NEXT)
    state_next = state + 1;
  else
    state_next = nextState;
  doDistanceInCM(distance, state_next);
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
  int state_next = 0;
  if(nextState == STATE_NEXT)
    state_next = state + 1;
  else
    state_next = nextState;
  doAngleRotation(angle, state_next);
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
  int state_next = 0;
  if(nextState == STATE_NEXT)
    state_next = state + 1;
  else
    state_next = nextState;
  doAngleRotation(angle, state_next);
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

char c1, c2;
int n_read, n_angle = 90, n_dist = 100, n_wait = 100, n_tdelay, n_pwm = 30;

void VisionBase::doLoop()
{   
  switch(state)
  {
    case 0: 
      if (Serial.available() > 0)
      {
        c1 = Serial.read();
        Serial.print("(");
        Serial.print(c1);
        Serial.print(") ");
        switch (c1)
        {
          case 'n':
            Serial.print("New value: ");
            c2 = Serial.read();Serial.print("(");Serial.print(c2);Serial.print(") ");
            n_read = Serial.parseInt();
            switch (c2)
            {
              case 'a':
                n_angle = n_read;
                Serial.print("n_angle set to ");
                break;
              case 'd':
                n_dist = n_read;
                Serial.print("n_distance set to ");
                break;
              case 'w':
                n_wait = n_read;
                Serial.print("n_wait set to ");
                break;
              case 't':
                n_tdelay = n_read;
                Serial.print("n_tdelay set to ");
                break;
              case 'p':
                n_pwm = n_read;
                Serial.print("n_pwm set to ");
                break;
              default:
                Serial.print("not recognized;\npossible values:\na - n_angle\nd - n_dist\nw - n_wait\nt - n_tdelay\np - n_pwm\nfollowed by a number\n");
                break;
            }
            Serial.print("#(");Serial.print(n_read);Serial.print(") ");
            break;
          case 'b':
            Serial.print("base ");
            c2 = Serial.read();Serial.print("(");Serial.print(c2);Serial.print(") ");
            switch (c2)
            {
              case 'f':
                Serial.print("moving forward by n_dist ");Serial.print(n_dist);
                Serial.print(" using pwm ");Serial.print(n_pwm);
                moveForward(n_dist, n_pwm, state);
                break;
              case 'l':
                Serial.print("turning left by n_angle ");Serial.print(n_angle);
                Serial.print(" using pwm ");Serial.print(n_pwm);
                turnLeft(n_angle, n_pwm, state);
                break;
              case 'r':
                Serial.print("turning right by n_angle ");Serial.print(n_angle);
                Serial.print(" using pwm ");Serial.print(n_pwm);
                turnRight(n_angle, n_pwm, state);
                break;
              case 'b':
                Serial.print("moving backward by n_dist ");Serial.print(n_dist);
                Serial.print(" using pwm ");Serial.print(n_pwm);
                moveBackward(n_dist, n_pwm, state);
                break;
              default:
                Serial.print("not recognized;\npossible values:f-forward,l-left,r-right,b-backward");
                break;
            }
            break;
          case 's':
            Serial.print("servo ");
            c2 = Serial.read();Serial.print("(");Serial.print(c2);Serial.print(") ");
            switch (c2)
            {
              case 'q':
                Serial.print("test 1 - grabbing claw then wait ");
                grabClaw();
                state.wait(n_wait,state);
                break;
              case 'w':
                Serial.print("test 2 - opening arms then wait ");
                openLeftArm();
                openRightArm();
                state.wait(n_wait,state);
                break;
              case 'e':
                Serial.print("test 3 - closing arms then wait ");
                closeLeftArm();
                closeRightArm();
                state.wait(n_wait,state);
                break;
              case 'r':
                Serial.print("test 4 - nothing ");
                break;
              case 't':
                Serial.print("test 5 - nothing ");
                break;
              default:
                Serial.print("not recognized;\npossible values:\nq-test1\nw-test2\ne-test3\nr-test4\nt-test5");
                break;
            }
            break;
          case 'i':
            Serial.print("Info - current values:\n");
            Serial.print("n_angle ");Serial.print(n_angle);Serial.print(" (degrees) - used for left and right base rotation\n");
            Serial.print("n_dist ");Serial.print(n_dist);Serial.print(" (cm) - used for forward and backward base movement\n");
            Serial.print("n_wait ");Serial.print(n_wait);Serial.print(" (ms) - used to wait after servo movements\n");
            Serial.print("n_pwm ");Serial.print(n_pwm);Serial.print(" (duty cycle 0-255) - pwm used for all base movements\n");
            Serial.print("n_tdelay ");Serial.print(n_tdelay);Serial.print(" (ms) - unused\n");
            break;
          default:
            Serial.print("not recognized;\npossible values:n-number,b-base,s-servo,i-info");
            break;
        }
        Serial.println();
      }
      break;
    case 1:
      turnRight(90, 30, STATE_NEXT);
      break;
    case 2: 
      openRightArm();
      moveForward(45, 30, STATE_NEXT);
      break;
    case 3:
      turnRight(90, 30, STATE_NEXT);
      break;
    case 4: 
      moveForward(15, 30, STATE_NEXT);
      break;
    case 5:
      turnLeft(90, 30,STATE_NEXT);
      break;
    case 6: 
      moveForward(20, 30,STATE_NEXT);
      break;
    case 7:
      grabRightArm();
      state.wait(300,STATE_NEXT);
      break;
    case 8: 
      moveForward(7, 30,STATE_NEXT);
      break;
    case 9:
      turnRight(90, 30,STATE_NEXT);
      break;
    case 10: 
     // openLeftClaw();
   //   openRightClaw();
      moveForward(21, 30,STATE_NEXT);
      break;
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
      turnRight(30, 30,STATE_NEXT);
      break;
    case 12: 
      moveBackward(12, 30,STATE_NEXT);
      break;
    case 13:
      turnLeft(30, 30,STATE_NEXT);
      break;
    case 14: 
      moveForward(17, 30,STATE_NEXT);
      break;
    case 15: 
      openLeftArm();
      state.wait(100,STATE_NEXT);
      break;
    case 16: 
      moveBackward(30, 30,STATE_NEXT);
      break;
    case 17: 
      closeLeftArm();
      state.wait(100,STATE_NEXT);
      break;
    case 18: 
      moveBackward(30, 30,STATE_NEXT);
      break;
    case 19: 
      openLeftArm();
      state.wait(100,STATE_NEXT);
      break;
    case 20: 
      moveBackward(30, 30,STATE_STOP);
      break;
    case STATE_STOP:
      break;
    default:
      state.doLoop();   
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
