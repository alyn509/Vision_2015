#ifndef pins_little_robot_h
#define pins_little_robot_h
  
/******************************************************* Sensors & buttons ********************************************************/

const int frontRightSensorPin = 99;
const int frontLeftSensorPin = 99;

const int startButtonPin = 2;
const int sideButtonPin = 53;

/******************************************************* Device Motors ********************************************************/

const int leftArmServoPin = 30;            // left arm               110 inside   -    30 clappers    -    90 popcorn glass 
const int rightArmServoPin = 24;           // right arm               20 inside   -   100 clappers    -    40 popcorn glass

const int clawServoPin = 28;           // Claw                        25 closed   -    100 opened
const int doorServoPin = 22;           // door opener                 70 closed   -   165 opened

const int leftPopCornHolderPin = 38;          // left popcorn holder     180 holding  -   130 released
const int rightPopCornHolderPin = 40;         // right popcorn holder      0 holding  -    60 released 

const int liftLimitatorSensorPin = 13;     // lift botton

const int popCornGrabberDCPin = 4;           // dc rotating continous

const int upLiftPin = 2;                     // bring lift up
const int downLiftPin = 3;                   // bring lift down

/******************************************************* Movement motors ********************************************************/
const int leftMotorFw = 10;
const int leftMotorBw = 11;

const int rightMotorFw = 8;
const int rightMotorBw = 9;

/******************************************************* Encoders ********************************************************/

const int leftEncoderStepPin = 6;
const int rightEncoderStepPin = 7;

#endif


