#include <elapsedMillis.h>
#include <Stepper.h>
#include <LiquidCrystal.h>
#include <Servo.h> 
#include "VisionDC.h"
#include "VisionEncoders.h"
#include "VisionBase.h"
#include "VisionState.h"
#include "pins.h"
#include "constants.h"

#define NINETYSECONDS 89000L

VisionBase base;
elapsedMillis timeUpTimer, enemyTimer;
boolean stoppedEverything = true; 
VisionState state;

VisionSensor startButton;
void setup()
{   
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);    // we set pin 3 as GND.. because of Claudiu
  
  startButton.initPin(startButtonPin);
  startButton.setAsPullup();
  while(startButton.detect());
  
  Serial.begin(115200);
  timeUpTimer = 0;
  base.init();
  delay(50);
  stoppedEverything = false;
}

char c1, c2;
int n_read, n_angle = 90, n_dist = 100, n_wait = 100, n_tdelay;
void loop()
{   
  switch (state)
  {  
    case 0:      
      base.update();
      state.waitMicros(10000, 0);
      break;
    default:
      state.doLoop();
  }   
  
  if(!stoppedEverything)
  {
    base.doLoop();
     
    switch(movementState)
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
                default:
                  Serial.print("not recognized;\npossible values:\na - n_angle\nd - n_dist\nw - n_wait\nt - n_tdelay\nfollowed by a number\n");
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
                  Serial.print("moving forward by n_dist ");
                  Serial.print(n_dist);
                  base.moveForward(n_dist, 4000);
                  movementState.waitFor(baseStop,movementState);
                  break;
                case 'l':
                  Serial.print("turning left by n_angle ");
                  Serial.print(n_angle);
                  base.turnLeft(n_angle);
                  movementState.waitFor(baseStop,movementState);
                  break;
                case 'r':
                  Serial.print("turning right by n_angle ");
                  Serial.print(n_angle);
                  base.turnRight(n_angle);
                  movementState.waitFor(baseStop,movementState);
                  break;
                case 'b':
                  Serial.print("moving backward by n_dist ");
                  Serial.print(n_dist);
                  base.moveBackward(n_dist, 4000);
                  movementState.waitFor(baseStop,movementState);
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
                  Serial.print("test 1 - grabbing claws, opening arms then wait ");
                  base.grabLeftClaw();
                  base.grabRightClaw();
                  base.openLeftArm();
                  base.openRightArm();
                  movementState.wait(n_wait,movementState);
                  break;
                case 'w':
                  Serial.print("test 2 - closing arms then wait ");
                  base.closeLeftArm();
                  base.closeRightArm();
                  movementState.wait(n_wait,movementState);
                  break;
                case 'e':
                  Serial.print("test 3 - nothing ");
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
        base.turnRight(90);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 2: 
        base.openRightArm();
        base.moveForward(45, 4000);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 3:
        base.turnRight(90);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 4: 
        base.moveForward(15, 4000);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 5:
        base.turnLeft(90);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 6: 
        base.moveForward(20, 4000);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 7:
        base.grabRightArm();
        movementState.wait(300,STATE_NEXT);
        break;
      case 8: 
        base.moveForward(5, 4000);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 9:
        base.turnRight(90);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 10: 
        base.moveForward(25, 4000);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 11: 
        base.grabLeftClaw();
        base.grabRightClaw();
        base.openLeftArm();
        movementState.wait(100,STATE_STOP);
        break;
      case 12: 
        base.moveBackward(20, 4000);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 13: 
        base.closeLeftArm();
        base.moveForward(20, 4000);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case 14: 
        base.openLeftArm();
        base.moveForward(20, 4000);
        movementState.waitFor(baseStop,STATE_STOP);
        break;
      case 15: 
        base.turnLeft(10);
        movementState.waitFor(baseStop,STATE_NEXT);
        break;
      case STATE_STOP:
        break;
      default:
        movementState.doLoop();   
    }
    checkForObstacle();
  }
  testIfTimeUp();
}

void testIfTimeUp()
{
  if(timeUpTimer == NINETYSECONDS)
    timeIsUpStopEverything();
}

void timeIsUpStopEverything()
{
  stoppedEverything = true;
  state = STATE_STOP;
  base.stopNow();
}

void checkForObstacle()
{
  if(!base.isStopped)
  {
    if(base.frontDetected() == true ) 
      enemyTimer = 0;
    if(base.frontDetected() == true && !base.isPaused && !base.ignoredSensors)   
      base.pause();
    else if(base.frontDetected() == false && base.isPaused && enemyTimer > 500L)
      base.unpause();
  }
}
