#include "VisionDC.h"

void VisionDC::init(int fwPin, int bwPin)
{
  forwardPin = fwPin;
  backwardPin = bwPin;
  
  pinMode(forwardPin, OUTPUT);
  pinMode(backwardPin, OUTPUT);
  
  digitalWrite(forwardPin, LOW);  
  digitalWrite(backwardPin, LOW);  
}

void VisionDC::moveForward(unsigned char pwm)
{
  analogWrite(forwardPin, pwm);
  digitalWrite(backwardPin, LOW);  
  isOn = true;
}
void VisionDC::moveBackward(unsigned char pwm)
{
  analogWrite(backwardPin, pwm);
  digitalWrite(forwardPin, LOW);  
  isOn = true;
}
void VisionDC::stopMotor()
{
  digitalWrite(forwardPin, LOW);  
  digitalWrite(backwardPin, LOW);  
  isOn = false;
}

