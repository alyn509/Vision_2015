#ifndef pins_little_robot_h
#define pins_little_robot_h
  
const int frontRightSensorPin = 10;
const int frontLeftSensorPin = 11;

const int startButtonPin = 2;

const int leftArmServoPin = 22;            // left arm               110 inside   -    30 clappers    -    90 popcorn glass 
const int rightArmServoPin = 24;           // right arm               20 inside   -   100 clappers    -    40 popcorn glass

const int leftClawServoPin = 32;           // leftClaw               150 opened   -    40 grabbed     -    12 closed
const int rightClawServoPin = 28;          // rightClaw                5 opened   -   115 grabbed     -   138 closed

const int leftLimitatorServoPin = 26;      // leftLimitator           13 holding  -    40 opened
const int rightLimitatorServoPin = 36;     // rightLimitator          97 holding  -    60 opened

const int leftDoorServoPin = 30;           // left door opener         0 closed   -   180 opened
const int rightDoorServoPin = 34;          // right door opener

const int leftPopCornHolderPin = 38;          // left popcorn holder     180 holding  -   130 released
const int rightPopCornHolderPin = 40;         // right popcorn holder      0 holding  -    60 released 

const int liftLimitatorSensorPin = 53;     // lift botton

const int popCornGrabberDCPin = 12;           // dc rotating continous

const int upLiftPin = A0;                     // bring lift up
const int downLiftPin = A1;                   // bring lift down
/*
const int leftMotorEnablePin = 50;
const int leftMotorStepPin = 52;
const int leftMotorDirectionPin = 48;

const int rightMotorEnablePin = 42;
const int rightMotorStepPin = 46;
const int rightMotorDirectionPin = 44;
*/
const int leftMotorFw = 999;
const int leftMotorBw = 999;

const int rightMotorFw = 999;
const int rightMotorBw = 999;

const int leftEncoderStepPin = A5;
const int rightEncoderStepPin = A3;

#endif


