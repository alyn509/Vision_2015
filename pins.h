#ifndef pins_little_robot_h
#define pins_little_robot_h
  
/******************************************************* Sensors & buttons ********************************************************/

const int frontRightSensorPin = 10;
const int frontLeftSensorPin = 11;

const int startButtonPin = 2;
const int sideButtonPin = 2;
const int greenLed = 999;

/******************************************************* Device Motors ********************************************************/

const int leftArmServoPin = 22;            // left arm               110 inside   -    30 clappers    -    90 popcorn glass 
const int rightArmServoPin = 24;           // right arm               20 inside   -   100 clappers    -    40 popcorn glass

const int clawServoPin = 28;          // Claw       
const int doorServoPin = 30;           // door opener       

const int leftPopCornHolderPin = 38;          // left popcorn holder     180 holding  -   130 released
const int rightPopCornHolderPin = 40;         // right popcorn holder      0 holding  -    60 released 

const int liftLimitatorSensorPin = 53;     // lift botton

const int popCornGrabberDCPin = 12;           // dc rotating continous

const int upLiftPin = A0;                     // bring lift up
const int downLiftPin = A1;                   // bring lift down


/******************************************************* Movement motors ********************************************************/
const int leftMotorFw = 10;
const int leftMotorBw = 11;

const int rightMotorFw = 8;
const int rightMotorBw = 9;

/******************************************************* Encoders ********************************************************/

const int leftEncoderStepPin = A5;
const int rightEncoderStepPin = A3;

#endif


